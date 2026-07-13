// Per-robot Durable Object: the coordination point for one robot's gateway
// connection, its attached browsers, the control-authority lease, and
// on-demand video fanout. See src/shared/contracts/relay-protocol.md for
// the full wire contract this implements.
//
// Phase 1: control/video fanout is real (sensor/diagnostics/control_authority
// passthrough, cmd_vel relay, video byte-fanout, viewer-count-driven
// video_subscribe/video_unsubscribe, robot_status online/offline) — this is
// the DO-side plumbing needed to verify the gateway end-to-end against a
// local `wrangler dev`.
//   Phase 2 — enabled Hibernation for real: socket bookkeeping moved off
//     instance fields (wiped on eviction) onto `ctx.getWebSockets(tag)`
//     lookups, which the runtime keeps alongside the hibernated socket
//     itself. Also: Worker-side deploy + auth wiring (see index.ts).
//   Phase 3 — control-authority lease: take_control/release_control/
//     lease_heartbeat, TTL alarm (`ctx.storage`, survives Hibernation),
//     lease_state broadcast, cmd_vel gating. Per-connection identity (there
//     was none before this phase) is a `connId` attached to each
//     browser:control socket via `serializeAttachment` at accept time —
//     tags can't be added post-accept, and instance fields don't survive
//     eviction, so the attachment + durable-storage lease record are the
//     two Hibernation-safe primitives this relies on.

import { DurableObject } from 'cloudflare:workers';

type Role = 'robot' | 'browser';
type Channel = 'control' | 'video';

const ROBOT_PATH = /^\/robot\/([^/]+)\/(control|video)$/;
const BROWSER_PATH = /^\/ws\/([^/]+)\/(control|video)$/;

// Each accepted socket is tagged with exactly one combined "role:channel"
// tag (e.g. "robot:control"). This is the sole piece of per-socket state:
// everything else is derived by calling `ctx.getWebSockets(tag)` on demand,
// which — unlike instance fields — survives the DO being evicted from
// memory between messages under WebSocket Hibernation. A freshly
// constructed RobotRelay after eviction has no idea any socket exists until
// it asks the runtime via tags.
function tagFor(role: Role, channel: Channel): string {
  return `${role}:${channel}`;
}

function parseTag(tag: string | undefined): [Role, Channel] | null {
  if (!tag) return null;
  const [role, channel] = tag.split(':');
  if (role !== 'robot' && role !== 'browser') return null;
  if (channel !== 'control' && channel !== 'video') return null;
  return [role, channel];
}

// Gateway -> DO control-channel message types forwarded verbatim to every
// attached browser control socket — relay-protocol.md's "DO -> Browser"
// table (passthrough row).
const TELEMETRY_PASSTHROUGH_TYPES = new Set([
  'sensor_scan', 'sensor_imu', 'sensor_odom', 'diagnostics', 'control_authority',
]);

// relay-protocol.md lease timing: 1.5s holder heartbeat, 5s TTL (~3 missed
// heartbeats of slack) — a full order of magnitude coarser than the
// robot-local 0.3s deadman / 0.4s rambla_safety backstops, which remain the
// fine-grained final authority regardless of lease state.
const LEASE_TTL_MS = 5000;

interface LeaseRecord {
  holderConnId: string;
  expiresAt: number;
}

interface LeaseAttachment {
  connId?: string;
}

export class RobotRelay extends DurableObject {
  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);

    const robotMatch = url.pathname.match(ROBOT_PATH);
    if (robotMatch) {
      return this.acceptSocket(request, 'robot', robotMatch[2] as Channel);
    }

    const browserMatch = url.pathname.match(BROWSER_PATH);
    if (browserMatch) {
      return this.acceptSocket(request, 'browser', browserMatch[2] as Channel);
    }

    return new Response('not found', { status: 404 });
  }

  private async acceptSocket(request: Request, role: Role, channel: Channel): Promise<Response> {
    if (request.headers.get('Upgrade') !== 'websocket') {
      return new Response('expected websocket upgrade', { status: 426 });
    }

    // Auth-handshake note (relay-protocol.md): a new validly-authed robot
    // connection is authoritative over a stale one — close any existing
    // socket for this exact role+channel before accepting the new one.
    if (role === 'robot') {
      for (const stale of this.ctx.getWebSockets(tagFor('robot', channel))) {
        stale.close();
      }
    }

    const pair = new WebSocketPair();
    const [client, server] = Object.values(pair);
    this.ctx.acceptWebSocket(server, [tagFor(role, channel)]);

    if (role === 'robot' && channel === 'control') {
      // Online/offline source of truth is the control-channel robot socket's
      // connection state, not a heartbeat (relay-protocol.md "Online/offline
      // signal") — accept (post-auth, already checked by the Worker) is
      // "online", independent of whether `hello` has arrived yet.
      this.broadcastToBrowserControlSockets({ type: 'robot_status', online: true });
      // Re-sync video state to a freshly (re)connected robot (Phase 4): the
      // video_subscribe trigger below only fires on a *browser* video-socket
      // 0->1 transition, which won't happen again if a viewer was already
      // attached before the robot dropped -- without this, the camera
      // subscription would silently never come back after a gateway
      // restart even though control/robot_status/lease all recover fine.
      if (this.ctx.getWebSockets(tagFor('browser', 'video')).length > 0) {
        this.sendToRobotControlSocket({ type: 'video_subscribe' });
      }
      // Same re-sync, for telemetry (Phase 5 follow-up): sensor/diagnostics
      // forwarding is now viewer-gated too (see telemetry_subscribe below),
      // so a reconnecting robot needs to be told a browser is already here
      // for the same reason the video re-sync above does.
      if (this.ctx.getWebSockets(tagFor('browser', 'control')).length > 0) {
        this.sendToRobotControlSocket({ type: 'telemetry_subscribe' });
      }
    } else if (role === 'browser' && channel === 'control') {
      // Per-connection identity for the lease (Phase 3): generate once at
      // accept time and persist it via serializeAttachment, since tags are
      // fixed at acceptWebSocket and can't be added later when a lease is
      // granted, and instance fields don't survive Hibernation eviction.
      const connId = crypto.randomUUID();
      server.serializeAttachment({ connId } satisfies LeaseAttachment);

      // Sensor/diagnostics telemetry is on-demand (Phase 5 follow-up,
      // mirrors the video_subscribe 0->1 pattern below): this socket is
      // already accepted (and thus already counted) by the time we check,
      // so "was empty" is "count === 1" now, not 0.
      const wasEmpty = this.ctx.getWebSockets(tagFor('browser', 'control')).length === 1;
      if (wasEmpty) this.sendToRobotControlSocket({ type: 'telemetry_subscribe' });

      // "once immediately to any browser on its own connect (so a late
      // joiner isn't stuck waiting on a delta)" — relay-protocol.md.
      const online = this.ctx.getWebSockets(tagFor('robot', 'control')).length > 0;
      server.send(JSON.stringify({ type: 'robot_status', online }));
      server.send(JSON.stringify(await this.currentLeaseStateMessage()));
    } else if (role === 'browser' && channel === 'video') {
      // This new socket is already accepted (and thus already counted) by
      // the time we check, so "was empty" is "count === 1" now, not 0.
      const wasEmpty = this.ctx.getWebSockets(tagFor('browser', 'video')).length === 1;
      if (wasEmpty) this.sendToRobotControlSocket({ type: 'video_subscribe' });
    }

    return new Response(null, { status: 101, webSocket: client });
  }

  async webSocketMessage(ws: WebSocket, message: string | ArrayBuffer): Promise<void> {
    const parsedTag = parseTag(this.ctx.getTags(ws)[0]);
    if (!parsedTag) return;
    const [role] = parsedTag;

    if (message instanceof ArrayBuffer) {
      // Video channel is pure binary, gateway -> DO -> browsers only — no
      // decode/re-encode, byte-fanout only (relay-protocol.md "Video-channel
      // framing"). Drop rather than buffer if nobody's watching.
      if (role === 'robot') this.broadcastBinaryToBrowserVideoSockets(message);
      return;
    }

    let parsed: Record<string, unknown>;
    try {
      parsed = JSON.parse(message);
    } catch {
      return;
    }
    const type = parsed.type;

    if (role === 'robot') {
      if (typeof type === 'string' && TELEMETRY_PASSTHROUGH_TYPES.has(type)) {
        this.broadcastToBrowserControlSockets(parsed);
      }
      // 'hello' is logged/validated only per relay-protocol.md — no gating
      // for M4 (single gateway build, no version skew expected).
      return;
    }

    // role === 'browser'
    if (type === 'cmd_vel') {
      // Only the current lease holder's connection may drive
      // (relay-protocol.md "cmd_vel gating"). cmd_vel rides at 20Hz and the
      // UI already self-gates on lease state, so a non-holder message here
      // is only a race or a misbehaving client — drop silently rather than
      // adding a 20Hz error stream.
      if (await this.isLeaseHolder(ws)) {
        this.sendToRobotControlSocket(parsed);
      }
    } else if (type === 'take_control') {
      await this.handleTakeControl(ws);
    } else if (type === 'release_control') {
      await this.handleReleaseControl(ws);
    } else if (type === 'lease_heartbeat') {
      await this.handleLeaseHeartbeat(ws);
    }
  }

  async webSocketClose(ws: WebSocket, _code: number, _reason: string, _wasClean: boolean): Promise<void> {
    const parsedTag = parseTag(this.ctx.getTags(ws)[0]);
    if (!parsedTag) return;
    const [role, channel] = parsedTag;

    if (role === 'robot' && channel === 'control') {
      this.broadcastToBrowserControlSockets({ type: 'robot_status', online: false });
    } else if (role === 'browser' && channel === 'control') {
      // Release path 2 of 3 (relay-protocol.md): the holder's socket
      // closing releases synchronously, without waiting out the TTL alarm.
      await this.releaseLeaseIfHeldBy(ws);
      // 1->0 teardown (Phase 5 follow-up). NOTE: `ws` itself is NOT
      // excluded from getWebSockets(tag) at this point -- verified via
      // direct inspection during this phase's testing, its readyState is
      // still CLOSING (2), not yet gone -- so it must be filtered out
      // explicitly rather than assuming a bare length===0 check ever
      // becomes true with exactly one attached viewer (this contradicted
      // an earlier assumption in the equivalent video check below, which
      // had the same bug -- fixed alongside this one).
      const stillAttached = this.ctx.getWebSockets(tagFor('browser', 'control'))
        .filter((s) => s !== ws);
      if (stillAttached.length === 0) {
        this.sendToRobotControlSocket({ type: 'telemetry_unsubscribe' });
      }
    } else if (role === 'browser' && channel === 'video') {
      const stillAttached = this.ctx.getWebSockets(tagFor('browser', 'video'))
        .filter((s) => s !== ws);
      if (stillAttached.length === 0) {
        this.sendToRobotControlSocket({ type: 'video_unsubscribe' });
      }
    }
  }

  // Alarm fires when the current lease's TTL lapses with no heartbeat
  // received since it was last set (`ctx.storage.setAlarm`, reset on every
  // grant/heartbeat) — release path 3 of 3. Using a storage-backed alarm
  // rather than an in-memory timer means expiry still fires correctly even
  // if the DO hibernates and is evicted between heartbeats.
  async alarm(): Promise<void> {
    const lease = await this.ctx.storage.get<LeaseRecord>('lease');
    if (!lease) return;
    if (lease.expiresAt <= Date.now()) {
      await this.ctx.storage.delete('lease');
      this.broadcastToBrowserControlSockets({ type: 'lease_state', held: false });
    } else {
      // Defensive: a grant/heartbeat always resets the alarm to match
      // expiresAt, so this shouldn't normally fire early, but reschedule
      // rather than dropping the lease if it somehow does.
      await this.ctx.storage.setAlarm(lease.expiresAt);
    }
  }

  private connIdFor(ws: WebSocket): string | undefined {
    const attachment = ws.deserializeAttachment() as LeaseAttachment | null;
    return attachment?.connId;
  }

  private async isLeaseHolder(ws: WebSocket): Promise<boolean> {
    const connId = this.connIdFor(ws);
    if (!connId) return false;
    const lease = await this.ctx.storage.get<LeaseRecord>('lease');
    return !!lease && lease.holderConnId === connId && lease.expiresAt > Date.now();
  }

  private async currentLeaseStateMessage(): Promise<Record<string, unknown>> {
    const lease = await this.ctx.storage.get<LeaseRecord>('lease');
    if (!lease || lease.expiresAt <= Date.now()) {
      return { type: 'lease_state', held: false };
    }
    return {
      type: 'lease_state',
      held: true,
      holder_id: lease.holderConnId,
      expires_in_ms: lease.expiresAt - Date.now(),
    };
  }

  private async handleTakeControl(ws: WebSocket): Promise<void> {
    const connId = this.connIdFor(ws);
    if (!connId) return;

    const now = Date.now();
    const existing = await this.ctx.storage.get<LeaseRecord>('lease');
    if (existing && existing.expiresAt > now && existing.holderConnId !== connId) {
      ws.send(JSON.stringify({ type: 'lease_denied' }));
      return;
    }

    const record: LeaseRecord = { holderConnId: connId, expiresAt: now + LEASE_TTL_MS };
    await this.ctx.storage.put('lease', record);
    await this.ctx.storage.setAlarm(record.expiresAt);
    ws.send(JSON.stringify({ type: 'lease_granted' }));
    this.broadcastToBrowserControlSockets({
      type: 'lease_state', held: true, holder_id: connId, expires_in_ms: LEASE_TTL_MS,
    });
  }

  private async handleReleaseControl(ws: WebSocket): Promise<void> {
    await this.releaseLeaseIfHeldBy(ws);
  }

  private async releaseLeaseIfHeldBy(ws: WebSocket): Promise<void> {
    const connId = this.connIdFor(ws);
    if (!connId) return;
    const existing = await this.ctx.storage.get<LeaseRecord>('lease');
    if (!existing || existing.holderConnId !== connId) return;
    await this.ctx.storage.delete('lease');
    await this.ctx.storage.deleteAlarm();
    this.broadcastToBrowserControlSockets({ type: 'lease_state', held: false });
  }

  private async handleLeaseHeartbeat(ws: WebSocket): Promise<void> {
    const connId = this.connIdFor(ws);
    if (!connId) return;

    const now = Date.now();
    const existing = await this.ctx.storage.get<LeaseRecord>('lease');
    if (!existing || existing.expiresAt <= now || existing.holderConnId !== connId) {
      ws.send(JSON.stringify({
        type: 'error', code: 'not_lease_holder', message: 'lease_heartbeat received without an active lease',
      }));
      return;
    }

    const record: LeaseRecord = { holderConnId: connId, expiresAt: now + LEASE_TTL_MS };
    await this.ctx.storage.put('lease', record);
    await this.ctx.storage.setAlarm(record.expiresAt);
    // No broadcast on heartbeat (relay-protocol.md) — broadcasting at 1.5Hz
    // to every browser would fight the point of Hibernation for no benefit;
    // only acquire/release/expire transitions are broadcast.
  }

  private sendToRobotControlSocket(message: Record<string, unknown>): void {
    const [robotSocket] = this.ctx.getWebSockets(tagFor('robot', 'control'));
    robotSocket?.send(JSON.stringify(message));
  }

  private broadcastToBrowserControlSockets(message: Record<string, unknown>): void {
    const payload = JSON.stringify(message);
    for (const ws of this.ctx.getWebSockets(tagFor('browser', 'control'))) {
      // A send can still race a close the runtime hasn't finished tearing
      // down yet — don't let one bad socket break the fanout to the rest.
      try {
        ws.send(payload);
      } catch {
        // no-op: the runtime's own bookkeeping (ctx.getWebSockets) is the
        // source of truth, nothing local to clean up here anymore.
      }
    }
  }

  private broadcastBinaryToBrowserVideoSockets(frame: ArrayBuffer): void {
    for (const ws of this.ctx.getWebSockets(tagFor('browser', 'video'))) {
      try {
        ws.send(frame);
      } catch {
        // no-op, see broadcastToBrowserControlSockets.
      }
    }
  }
}
