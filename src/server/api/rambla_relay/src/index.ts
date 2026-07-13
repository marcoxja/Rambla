// Worker entry point for the M4 hosted relay (CTL-009). Terminates both
// auth layers (robot bearer token, browser basic auth — relay-protocol.md's
// "Auth handshake" sections), serves the reused static UI, and routes
// validated WebSocket upgrades to the per-robot Durable Object. Holds no
// protocol state itself — all fanout, lease, and connection state lives in
// RobotRelay (./robot-do.ts).
//
// Phase 2: static-UI serving is real (env.ASSETS, gated behind the same
// basic-auth check as /ws/*, run_worker_first so this handler always sees
// the request first — see wrangler.jsonc).

export { RobotRelay } from './robot-do';

export interface Env {
  ROBOT_RELAY: DurableObjectNamespace;
  ASSETS: Fetcher;
  ROBOT_TOKEN: string;
  BASIC_AUTH_USER: string;
  BASIC_AUTH_PASS: string;
}

type Channel = 'control' | 'video';

const ROBOT_PATH = /^\/robot\/([^/]+)\/(control|video)$/;
const BROWSER_PATH = /^\/ws\/([^/]+)\/(control|video)$/;

export default {
  async fetch(request: Request, env: Env): Promise<Response> {
    const url = new URL(request.url);

    const robotMatch = url.pathname.match(ROBOT_PATH);
    if (robotMatch) {
      return handleRobotConnect(request, env, robotMatch[1], robotMatch[2] as Channel);
    }

    const browserMatch = url.pathname.match(BROWSER_PATH);
    if (browserMatch) {
      return handleBrowserConnect(request, env, browserMatch[1], browserMatch[2] as Channel);
    }

    // Everything else is the static UI (index.html/css/js reused in place
    // from rambla_control_panel/static/, see wrangler.jsonc) — same
    // basic-auth gate as /ws/*, since this is the read-only-viewing layer
    // (relay-protocol.md layer 2), not just an unauthenticated landing page.
    if (!isValidBasicAuth(request, env)) {
      return new Response('unauthorized', {
        status: 401,
        headers: { 'WWW-Authenticate': 'Basic realm="rambla"' },
      });
    }
    return env.ASSETS.fetch(request);
  },
} satisfies ExportedHandler<Env>;

function handleRobotConnect(
  request: Request, env: Env, robotId: string, channel: Channel,
): Response | Promise<Response> {
  if (!isValidRobotToken(request, env)) {
    return new Response('unauthorized', { status: 401 });
  }
  return forwardToRelay(request, env, robotId);
}

function handleBrowserConnect(
  request: Request, env: Env, robotId: string, channel: Channel,
): Response | Promise<Response> {
  if (!isValidBasicAuth(request, env)) {
    return new Response('unauthorized', {
      status: 401,
      headers: { 'WWW-Authenticate': 'Basic realm="rambla"' },
    });
  }
  return forwardToRelay(request, env, robotId);
}

function forwardToRelay(request: Request, env: Env, robotId: string): Promise<Response> {
  const id = env.ROBOT_RELAY.idFromName(robotId);
  const stub = env.ROBOT_RELAY.get(id);
  return stub.fetch(request);
}

// Layer 1 (relay-protocol.md): robot presents `Authorization: Bearer
// <RAMBLA_ROBOT_TOKEN>` on the WSS upgrade request.
function isValidRobotToken(request: Request, env: Env): boolean {
  const auth = request.headers.get('Authorization');
  return auth === `Bearer ${env.ROBOT_TOKEN}`;
}

// Layer 2 (relay-protocol.md): standard HTTP Basic Auth, gates read-only
// viewing. Grants no drive authority by itself — see the control-authority
// lease (layer 3), which lives entirely in RobotRelay.
function isValidBasicAuth(request: Request, env: Env): boolean {
  const auth = request.headers.get('Authorization');
  if (!auth?.startsWith('Basic ')) return false;
  const decoded = atob(auth.slice('Basic '.length));
  const sep = decoded.indexOf(':');
  if (sep === -1) return false;
  const user = decoded.slice(0, sep);
  const pass = decoded.slice(sep + 1);
  return user === env.BASIC_AUTH_USER && pass === env.BASIC_AUTH_PASS;
}
