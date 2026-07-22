// Two independent sockets against the M4 hosted relay (relay-protocol.md) —
// a control channel (cmd_vel out; sensor/diagnostics/control_authority/
// robot_status in, multiplexed by a "type" field) and a video channel
// (binary JPEG frames only, no envelope). Separate sockets so a large video
// frame can never head-of-line-block a control message. `ROBOT_ID` is
// hardcoded: M4 ships with exactly one configured robot and the UI doesn't
// pick one (relay-protocol.md "Transport shape").
const RamblaWS = (() => {
  const ROBOT_ID = 'robot-1';

  let controlSocket = null;
  let videoSocket = null;
  const listeners = {};
  const connectionListeners = [];
  const videoFrameListeners = [];

  function wsUrl(channel) {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    return `${proto}://${location.host}/ws/${ROBOT_ID}/${channel}`;
  }

  function connectControl() {
    controlSocket = new WebSocket(wsUrl('control'));

    controlSocket.addEventListener('open', () => {
      connectionListeners.forEach((fn) => fn(true));
    });
    controlSocket.addEventListener('close', () => {
      connectionListeners.forEach((fn) => fn(false));
      setTimeout(connectControl, 1000);
    });
    controlSocket.addEventListener('error', () => controlSocket.close());
    controlSocket.addEventListener('message', (event) => {
      let msg;
      try {
        msg = JSON.parse(event.data);
      } catch {
        return;
      }
      // Passthrough payloads keep the gateway's `{type, data}` envelope
      // verbatim (relay-protocol.md "preserved"); listeners still get just
      // the inner data, matching debug-tabs.js's pre-M4 expectations. A few
      // new M4 message types (robot_status, control_authority, lease_*)
      // carry their fields at the top level instead - fall back to the
      // whole message for those so `on('robot_status', ...)` etc. still work.
      (listeners[msg.type] || []).forEach((fn) => fn(msg.data !== undefined ? msg.data : msg));
    });
  }

  function connectVideo() {
    videoSocket = new WebSocket(wsUrl('video'));
    videoSocket.binaryType = 'blob';

    videoSocket.addEventListener('close', () => setTimeout(connectVideo, 1000));
    videoSocket.addEventListener('error', () => videoSocket.close());
    videoSocket.addEventListener('message', (event) => {
      videoFrameListeners.forEach((fn) => fn(event.data));
    });
  }

  function on(type, fn) {
    listeners[type] = listeners[type] || [];
    listeners[type].push(fn);
  }

  function onConnectionChange(fn) {
    connectionListeners.push(fn);
  }

  function onVideoFrame(fn) {
    videoFrameListeners.push(fn);
  }

  function sendCmdVel(linear, angular) {
    if (controlSocket && controlSocket.readyState === WebSocket.OPEN) {
      controlSocket.send(JSON.stringify({ type: 'cmd_vel', linear, angular }));
    }
  }

  // take_control / release_control / lease_heartbeat (relay-protocol.md) —
  // all three are bare `{type}` messages with no payload.
  function sendControlMessage(type) {
    if (controlSocket && controlSocket.readyState === WebSocket.OPEN) {
      controlSocket.send(JSON.stringify({ type }));
    }
  }

  connectControl();
  connectVideo();

  return { on, onConnectionChange, onVideoFrame, sendCmdVel, sendControlMessage };
})();
