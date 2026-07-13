# rambla-relay

Cloudflare Worker + per-robot Durable Object — the hosted side of M4
(CTL-009). No ROS 2/Python here by design (`M4_PLAN.md`'s "Repository /
deployment boundary" — this is Cloudflare-native TypeScript; the robot-local
`rambla_control_panel` package is the gateway this talks to).

Wire contract: `../../../shared/contracts/relay-protocol.md`.

## Status

Phase 2 code-complete, deploy pending. Routing, both auth layers (robot
bearer token, browser basic auth), control/video fanout, viewer-count-driven
video subscribe/unsubscribe, and `robot_status` online/offline are all real.
The DO's socket bookkeeping is hibernation-safe: it's derived from
`ctx.getWebSockets(tag)` rather than instance fields, so state survives
eviction between messages. Static UI (reused in place from
`rambla_control_panel/static/`, see `wrangler.jsonc`'s `assets` block) is
served by the Worker behind the same basic-auth gate as the WebSocket
routes. Not yet implemented: the control-authority lease (Phase 3 — see the
`TODO(Phase 3)` markers in `src/robot-do.ts`) and confirmed video/control
isolation under load + graceful offline UI (Phase 4).

Deployed: `https://rambla-relay.jackdemarco-jd.workers.dev`. Verified
end-to-end against the sim VM's gateway and a real browser (connection
indicator, camera feed, joystick drive all confirmed working through the
live deployment).

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
