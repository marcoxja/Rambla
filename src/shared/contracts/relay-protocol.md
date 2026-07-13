# Relay Protocol Contract

Wire protocol for the M4 hosted-relay boundary (`M4_PLAN.md`, CTL-008/009/010,
INT-005): the robot-local gateway ⇄ Cloudflare Worker/Durable Object ⇄
browser link. Companion to `robot-interface.md`, which stays the canonical
ROS 2 contract — this doc starts where ROS 2 stops, at the gateway's outbound
WebSocket boundary. Nothing here changes `robot-interface.md`'s topic table;
the gateway is a client of it, not a new producer, except where noted below
(`/control_authority`).

Phase 0 (this doc): protocol frozen, nothing implemented. Phase 1 implements
the gateway side against a local `wrangler dev` DO; Phase 2 deploys; Phase 3
adds the lease; Phase 4 finishes video isolation + offline handling.

---

## Transport shape

Two independent outbound WSS connections, established and held open for the
gateway's lifetime (reconnected with backoff on drop — see below), plus the
mirrored pair on the browser side:

| | Robot (gateway) ⇄ Worker/DO | Browser ⇄ Worker/DO |
|---|---|---|
| Control | `wss://<host>/robot/{robot_id}/control` | `wss://<host>/ws/{robot_id}/control` |
| Video | `wss://<host>/robot/{robot_id}/video` | `wss://<host>/ws/{robot_id}/video` |

`{robot_id}` exists for future multi-robot routing (one DO instance per id,
via `idFromName(robot_id)`); M4 ships with exactly one configured robot, and
the static UI defaults to it without the user picking one.

Control and video are separate sockets on both hops specifically so a large
JPEG frame can never head-of-line-block a control/lease message (M4_PLAN.md
"video must not delay control/safety"). Nothing on the video channel is ever
control-bearing, and nothing on the control channel is ever binary image
data — see "Video-channel framing" below for the one exception that proves
this (`video_subscribe`/`video_unsubscribe`, which rides control, not video,
precisely to keep video purely-binary and one-directional).

The gateway keeps both connections open for its whole lifetime once
connected; "on-demand" (M4_PLAN.md) refers to the **ROS subscription** to
`/camera/image_raw/compressed`, not the WSS connection itself — the gateway
subscribes only while the DO has ≥1 attached browser video-socket and
unsubscribes when it drops to 0 (see `video_subscribe`/`video_unsubscribe`).
An idle open video WSS carrying zero frames costs nothing extra.

---

## Auth handshake (layer 1 — robot ⇄ Worker/DO)

The gateway presents `Authorization: Bearer <RAMBLA_ROBOT_TOKEN>` as a
request header on both outbound WSS upgrade requests (a real HTTP client,
unlike a browser, can set this). The Worker validates the token against a
secret binding (`wrangler secret put ROBOT_TOKEN`) before forwarding the
upgrade to the robot's DO. A missing/invalid token gets `401` and the
upgrade never reaches the DO.

The DO accepts **exactly one** control-channel robot connection and **exactly
one** video-channel robot connection at a time. If a new, validly-authed
robot connection arrives while an old one is still open (e.g. the gateway
restarted without a clean close), the DO treats the new connection as
authoritative: it closes the stale socket and adopts the new one. This is
what makes gateway auto-reconnect (Phase 4) safe — a reconnect always wins
over a zombie socket rather than being rejected as a duplicate.

On connect, the gateway's first control-channel message is `hello` (see
below); the DO logs/validates `protocol_version` but does not gate the
connection on it for M4 (single gateway build, no version skew expected —
kept as a forward-compatible hook only).

## Auth handshake (layer 2 — browser ⇄ Worker)

Standard HTTP Basic Auth, credentials checked against a Worker secret
binding. The Worker challenges (`401` + `WWW-Authenticate: Basic`) on the
static UI's initial load; the browser then attaches the cached
`Authorization: Basic <...>` header automatically to same-origin requests,
including the `/ws/{robot_id}/control` and `/ws/{robot_id}/video` upgrade
requests, so no separate login step is needed for the sockets. This gates
**read-only viewing** — diagnostics and camera. It grants no drive authority
by itself; see layer 3.

## Control-authority lease (layer 3 — CTL-010)

A **separate, explicit** action on top of layer 2, deliberately harder to
reach than viewing. The DO holds the lease state; it is not a token the
browser can forge or infer.

- **Holder identity** = the browser's control-channel WebSocket connection
  itself (the DO assigns an internal connection id on accept). No user
  accounts exist in M4 — "who holds control" is "which currently-open
  connection", which is sufficient to make the lease exclusive and to let
  the DO release it immediately on that connection's close.
- **Acquire:** browser sends `take_control`. If unheld (or the prior
  holder's TTL has lapsed), the DO grants it: reply `lease_granted` to the
  requester, broadcast `lease_state` to everyone. If held by a different
  live connection, reply `lease_denied` to the requester; no broadcast.
- **Heartbeat:** the holder sends `lease_heartbeat` every **1.5 s** while
  holding the lease. Each heartbeat resets the DO's lease-expiry alarm to
  **now + 5 s** (`state.storage.setAlarm`, so this works correctly under
  Hibernation — the DO doesn't need a live in-memory timer to wake up for
  expiry, the platform wakes it). 5 s / 1.5 s gives roughly 3 missed
  heartbeats of slack for ordinary network jitter before expiry, while
  staying a full order of magnitude coarser than the robot-local 0.3 s
  (deadman) / 0.4 s (`rambla_safety`) backstops — this lease is the outer,
  coarse bound; the robot-local timeouts remain the fine-grained final
  authority regardless of relay/lease state (defense in depth, per
  `M4_PLAN.md`).
- **Release — three paths, all immediate (no need to wait out the TTL):**
  1. Explicit `release_control` from the current holder.
  2. The holder's control-channel WebSocket closes (tab closed, network
     drop) — the DO's close handler releases synchronously.
  3. TTL alarm fires with no heartbeat received since it was last set.

  Every release broadcasts `lease_state` to all connected browsers.
- **`cmd_vel` gating:** the DO only forwards a `cmd_vel` message upstream to
  the gateway if it arrives on the connection that currently holds the
  lease. A `cmd_vel` from any other connection is dropped (optionally with
  an `error` reply to that sender) — never forwarded, never merged.

None of this replaces the robot-local boundary. Even a correctly-leased,
correctly-heartbeating browser still only ever reaches
`/cmd_vel_teleop`; `rambla_safety`'s AUTO/MANUAL state machine and collision
filter apply exactly as documented in `robot-interface.md` regardless of
what the lease says.

---

## Control-channel messages

JSON text frames, one object per frame, always shaped `{"type": "<type>",
...}` — the same envelope convention the current `ws-client.js` already uses
for `/ws`, just extended with new types and a hop in the middle.

### Gateway → DO

| `type` | Fields | Notes |
|---|---|---|
| `hello` | `robot_id`, `protocol_version` | Sent once, immediately after connect. |
| `sensor_scan` | same payload shape as today's `on_scan` sink | Forwarded verbatim at `SENSOR_PUBLISH_RATE_HZ` (8 Hz), **on-demand only** (Phase 5 follow-up — see `telemetry_subscribe` below). |
| `sensor_imu` | same payload shape as today's `on_imu` sink | ″ |
| `sensor_odom` | same payload shape as today's `on_odom` sink | ″ |
| `diagnostics` | same payload shape as today's `on_diagnostics` sink | 1 Hz, **on-demand only** — same gate as the sensor rows. |
| `control_authority` | `mode`: `"AUTO"` \| `"MANUAL"` | Gateway subscription to `/control_authority` (`robot-interface.md`), added in Phase 1. Edge-triggered and cheap regardless of viewers — **not** gated by `telemetry_subscribe`. |

### DO → Gateway

| `type` | Fields | Notes |
|---|---|---|
| `cmd_vel` | `linear`, `angular` | Only ever forwarded by the DO from the current lease holder (layer 3); gateway calls the existing `DeadmanTimer.on_command` unchanged. |
| `video_subscribe` | — | Sent on the DO's browser-side video-viewer count going 0→1 (or on a robot control-socket reconnect if a viewer is already attached — Phase 4 fix). Gateway creates the `/camera/image_raw/compressed` subscription. |
| `video_unsubscribe` | — | Sent on viewer count going 1→0. Gateway destroys the subscription. |
| `telemetry_subscribe` | — | **Phase 5 follow-up.** Sent on the DO's browser-side *control*-socket count going 0→1, or on a robot control-socket reconnect if a browser control socket is already attached (same reconnect-resync shape as `video_subscribe`). Gateway creates the `/scan`, `/imu/data_fixed`, `/odometry/filtered` subscriptions and resumes the diagnostics/liveness timers. Measured (M4_PLAN.md Phase 5) at ~80% mean CPU always-on regardless of viewers — the raw sensor callbacks (IMU is ~95 Hz) and the 1 Hz whole-graph liveness probe were never gated by anything, unlike the camera path M4 was built around. |
| `telemetry_unsubscribe` | — | Sent on browser control-socket count going 1→0. Gateway destroys the sensor subscriptions and stops the diagnostics/liveness timers. |

### Browser → DO

| `type` | Fields | Notes |
|---|---|---|
| `cmd_vel` | `linear`, `angular` | Same shape the existing UI already sends; only honored from the lease holder. |
| `take_control` | — | Request the exclusive lease. |
| `release_control` | — | Voluntarily release the lease. |
| `lease_heartbeat` | — | Sent every 1.5 s by the current holder. |

### DO → Browser

| `type` | Fields | Notes |
|---|---|---|
| `sensor_scan` / `sensor_imu` / `sensor_odom` / `diagnostics` | passthrough | Fanned out to every connected browser verbatim, unchanged shape from today's UI expectations. Only arrives at all while `telemetry_subscribe` is active on the gateway side — see the Gateway → DO table. |
| `control_authority` | `mode` | Passthrough of the gateway's new subscription. |
| `robot_status` | `online`: bool | Pushed on every robot control-socket connect/disconnect, and once immediately to any browser on its own connect (so a late joiner isn't stuck waiting on a delta). |
| `lease_state` | `held`: bool, `holder_id?`, `expires_in_ms?` | Broadcast on every acquire/release/expire, and once immediately on a new browser connect. |
| `lease_granted` | — | Direct reply to the requester's `take_control`. |
| `lease_denied` | — | Direct reply to the requester's `take_control` when already held elsewhere. |
| `error` | `code`, `message` | e.g. rejected `cmd_vel` from a non-holder, malformed frame. |

---

## Video-channel framing

Binary WebSocket frames only — no JSON envelope, no type tag. Each frame's
payload is exactly `CompressedImage.data` (JPEG bytes, q85, 640×480, per
`robot-interface.md`), forwarded byte-for-byte:

`gateway subscription callback → binary frame over gateway's video WSS → DO
relays the same bytes, unchanged, to every attached browser video WSS`.

The DO does no decoding, re-encoding, or inspection of frame contents — it
is a byte-fanout, matching the "never decode/re-encode on the hosted side
either" spirit of the gateway-side change (the whole point of M4 is that
*nobody* between the camera and the browser touches pixels). If a frame
arrives while the DO has zero attached video viewers (a momentary race on
unsubscribe), the DO drops it rather than buffering.

---

## Online/offline signal

The DO's source of truth for "is the robot online" is the **control-channel
robot socket's connection state** — not a heartbeat message, not the video
socket. `onOpen` (after auth) → online; `onClose` (any reason: clean close,
error, timeout) → offline, broadcast immediately as `robot_status` to every
attached browser. The UI (Phase 4) shows an explicit offline state and
disables controls on this signal — never a hung "connecting..." state.

The gateway independently reconnects outbound with backoff on any drop of
either socket; per the auth-handshake note above, a reconnect always
displaces a stale DO-side socket rather than being rejected.

---

## Summary: what's new vs what's preserved

**Preserved verbatim** (payload shapes carry through unchanged, only the
transport hop changes): `sensor_scan`/`sensor_imu`/`sensor_odom` snapshot
payloads, `diagnostics` payload, the `cmd_vel {linear, angular}` shape the
existing UI already sends, `/cmd_vel_teleop` as the sole gateway publish
target, `DeadmanTimer` semantics untouched.

**New for M4:** the `hello` handshake; `/control_authority` forwarding
(gateway subscribes to a topic it didn't before); `video_subscribe` /
`video_unsubscribe` (viewer-count-driven subscription lifecycle);
`robot_status` (online/offline); the entire lease message set
(`take_control` / `release_control` / `lease_heartbeat` / `lease_granted` /
`lease_denied` / `lease_state`); binary-only video framing (replacing the
MJPEG multipart stream).
