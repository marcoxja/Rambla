// Control-authority lease UI (relay-protocol.md layer 3 / CTL-010). Default
// read-only; "Take control" is a separate, harder-to-reach action than
// viewing. Owns the single `iHoldControl` flag that gates the joysticks
// (joystick.js) and the 1.5s heartbeat that keeps a held lease alive.
//
// This is a coordination/UX layer only — losing the lease (release, denial,
// expiry, disconnect) always goes silent rather than streaming zeros, same
// as the pre-M4 deadman contract, so the robot-local deadman (0.3s) +
// rambla_safety (0.4s) backstops remain the real final authority regardless
// of what this module or the relay's lease state say.
const LEASE_HEARTBEAT_MS = 1500;

let iHoldControl = false;
let heartbeatTimer = null;
let robotOnline = false;
let lastLeaseState = { held: false };

function stopHeartbeat() {
  if (heartbeatTimer) {
    clearInterval(heartbeatTimer);
    heartbeatTimer = null;
  }
}

function setHolding(holding) {
  iHoldControl = holding;
  setControlEnabled(holding);
  if (holding) {
    stopHeartbeat();
    heartbeatTimer = setInterval(() => {
      RamblaWS.sendControlMessage('lease_heartbeat');
    }, LEASE_HEARTBEAT_MS);
  } else {
    stopHeartbeat();
  }
}

function renderButton() {
  const btn = document.getElementById('take-control-btn');
  const status = document.getElementById('control-status');
  if (!btn || !status) return;

  if (!robotOnline) {
    btn.disabled = true;
    btn.textContent = 'Take control';
    status.textContent = 'Robot offline';
    status.className = 'offline';
  } else if (iHoldControl) {
    btn.disabled = false;
    btn.textContent = 'Release control';
    status.textContent = 'You are driving';
    status.className = 'driving';
  } else if (lastLeaseState.held) {
    btn.disabled = true;
    btn.textContent = 'Take control';
    status.textContent = 'Another user is driving';
    status.className = 'held-elsewhere';
  } else {
    btn.disabled = false;
    btn.textContent = 'Take control';
    status.textContent = 'Read-only';
    status.className = 'available';
  }
}

function initControlLease() {
  const btn = document.getElementById('take-control-btn');
  btn.addEventListener('click', () => {
    if (iHoldControl) {
      RamblaWS.sendControlMessage('release_control');
      // Go silent immediately on our own end rather than waiting for the
      // lease_state echo back — matches "release on disconnect/close is
      // immediate" everywhere else in this contract.
      setHolding(false);
      renderButton();
    } else {
      RamblaWS.sendControlMessage('take_control');
    }
  });

  RamblaWS.on('robot_status', (msg) => {
    robotOnline = !!msg.online;
    if (!robotOnline && iHoldControl) setHolding(false);
    renderButton();
  });

  RamblaWS.on('lease_granted', () => {
    setHolding(true);
    renderButton();
  });

  RamblaWS.on('lease_denied', () => {
    renderButton();
  });

  RamblaWS.on('lease_state', (msg) => {
    lastLeaseState = msg;
    // held:false always means we lost it too, if we thought we had it.
    // Acquiring is confirmed only via the direct `lease_granted` reply
    // above, never inferred from this broadcast.
    if (!msg.held && iHoldControl) setHolding(false);
    renderButton();
  });

  RamblaWS.on('error', (msg) => {
    if (msg.code === 'not_lease_holder' && iHoldControl) {
      setHolding(false);
      renderButton();
    }
  });

  RamblaWS.onConnectionChange((connected) => {
    if (!connected && iHoldControl) setHolding(false);
    if (!connected) {
      robotOnline = false;
      lastLeaseState = { held: false };
    }
    renderButton();
  });

  renderButton();
}
