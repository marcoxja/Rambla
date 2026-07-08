// Single shared websocket for cmd_vel (outbound) and sensor/diagnostics
// (inbound), multiplexed by a "type" field - see server.py's docstring for
// why one connection instead of one per tab.
const RamblaWS = (() => {
  let socket = null;
  const listeners = {};
  const connectionListeners = [];

  function connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    socket = new WebSocket(`${proto}://${location.host}/ws`);

    socket.addEventListener('open', () => {
      connectionListeners.forEach((fn) => fn(true));
    });
    socket.addEventListener('close', () => {
      connectionListeners.forEach((fn) => fn(false));
      setTimeout(connect, 1000);
    });
    socket.addEventListener('error', () => socket.close());
    socket.addEventListener('message', (event) => {
      let msg;
      try {
        msg = JSON.parse(event.data);
      } catch {
        return;
      }
      (listeners[msg.type] || []).forEach((fn) => fn(msg.data));
    });
  }

  function on(type, fn) {
    listeners[type] = listeners[type] || [];
    listeners[type].push(fn);
  }

  function onConnectionChange(fn) {
    connectionListeners.push(fn);
  }

  function sendCmdVel(linear, angular) {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify({ type: 'cmd_vel', linear, angular }));
    }
  }

  connect();

  return { on, onConnectionChange, sendCmdVel };
})();
