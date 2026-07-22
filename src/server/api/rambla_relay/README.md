# rambla-relay

Cloudflare Worker + per-robot Durable Object — the hosted side of M4
(CTL-009). No ROS 2/Python here by design: this is Cloudflare-native
TypeScript that reimplements the old on-VM FastAPI server's multiplex/
broadcast logic, direction inverted (the robot dials out instead of the
browser dialing in). The robot-local `rambla_control_panel` package is the
gateway this talks to.

Wire contract: `../../../shared/contracts/relay-protocol.md`.

## Status

**M4 complete.** Deployed: `https://rambla-relay.jackdemarco-jd.workers.dev`.
Everything in `relay-protocol.md` is implemented and verified live: both
auth layers (robot bearer token, browser basic auth), the control-authority
lease (exclusive single-holder, TTL + heartbeat via a DO Alarm, three release
paths), control/video fanout on two independent WebSockets, viewer-count-
driven video *and* telemetry subscribe/unsubscribe (camera and
`/scan`/`/imu`/`/odom`/diagnostics both on-demand, gated on browser
video-socket and control-socket counts respectively), `robot_status`
online/offline with gateway reconnect-with-backoff, and re-sync of video/
telemetry subscriptions on a robot reconnect while a viewer is still
attached.

The DO's socket bookkeeping is hibernation-safe: it's derived from
`ctx.getWebSockets(tag)` rather than instance fields, so state survives
eviction between messages. Static UI (reused in place from
`rambla_control_panel/static/`, see `wrangler.jsonc`'s `assets` block) is
served by the Worker behind the same basic-auth gate as the WebSocket
routes.

Verified live against the deployed Worker + the sim VM's gateway and a real
browser: connection indicator, camera feed, joystick drive, take/release
control, a second browser correctly denied while another holds the lease,
robot offline/reconnect recovery (including the camera feed resuming without
a manual restart), and video-channel saturation not delaying control-channel
messages (0.6-0.8ms mean / ≤4ms max latency under a fully-saturated video
socket). See `robot/resource-budget/CLAUDE.md`'s 2026-07-12 measurement log
for the idle-CPU numbers this milestone was built to fix (79% mean → 21%
mean, once on-demand gating was extended from the camera to sensor/
diagnostics telemetry).

Not yet captured: 60s bandwidth samples for the viewing and driving+viewing
scenarios (subscribe/unsubscribe behavior for both was verified directly,
just not via a full `sample_resources.py` capture — see the resource-budget
measurement log for the reasoning and estimated numbers).

## Local development

```sh
npm install
cp .dev.vars.example .dev.vars   # edit if you want non-default local creds
npm run dev                       # wrangler dev
```

Phase 1 points the gateway's outbound WSS client at this local `wrangler
dev` instance before anything is deployed to Cloudflare.

## Deploying (Phase 2+)

```sh
wrangler secret put ROBOT_TOKEN
wrangler secret put BASIC_AUTH_USER
wrangler secret put BASIC_AUTH_PASS
npm run deploy
```
