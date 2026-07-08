// Two independent virtual joysticks (translation, rotation) - one axis
// each, matching linear.x / angular.z 1:1 with no diagonal-vector math.
// Sends at 20Hz over the shared websocket (matches the server's cmd_vel
// republish rate and the -r 20 rate of the ros2 topic pub workaround this
// replaces).
const CMD_VEL_RATE_MS = 1000 / 20;
const MAX_LINEAR = 0.5;   // m/s - conservative for a sim apartment world
const MAX_ANGULAR = 1.5;  // rad/s

function setupJoystick(canvas) {
  const ctx = canvas.getContext('2d');
  const axis = canvas.dataset.axis; // 'linear' or 'angular'
  let value = 0; // -1..1
  let dragging = false;

  function resize() {
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * devicePixelRatio;
    canvas.height = rect.height * devicePixelRatio;
  }

  function draw() {
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    const cx = w / 2, cy = h / 2, r = Math.min(w, h) / 2 - 4 * devicePixelRatio;

    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(255,255,255,0.3)';
    ctx.lineWidth = 2 * devicePixelRatio;
    ctx.stroke();

    const knobOffset = axis === 'linear' ? -value * r * 0.7 : value * r * 0.7;
    const kx = axis === 'linear' ? cx : cx + knobOffset;
    const ky = axis === 'linear' ? cy + knobOffset : cy;

    ctx.beginPath();
    ctx.arc(kx, ky, r * 0.35, 0, Math.PI * 2);
    ctx.fillStyle = dragging ? '#4da3ff' : 'rgba(255,255,255,0.6)';
    ctx.fill();
  }

  function valueFromPointer(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const r = Math.min(rect.width, rect.height) / 2;
    if (axis === 'linear') {
      return Math.max(-1, Math.min(1, -(clientY - cy) / r));
    }
    return Math.max(-1, Math.min(1, (clientX - cx) / r));
  }

  canvas.addEventListener('pointerdown', (e) => {
    dragging = true;
    canvas.setPointerCapture(e.pointerId);
    value = valueFromPointer(e.clientX, e.clientY);
    draw();
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    value = valueFromPointer(e.clientX, e.clientY);
    draw();
  });
  function release(e) {
    if (!dragging) return;
    dragging = false;
    value = 0;
    draw();
  }
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);

  window.addEventListener('resize', () => { resize(); draw(); });
  resize();
  draw();

  return {
    getValue: () => value,
  };
}

function initJoysticks() {
  const leftStick = setupJoystick(document.getElementById('joystick-left'));
  const rightStick = setupJoystick(document.getElementById('joystick-right'));

  setInterval(() => {
    const linear = leftStick.getValue() * MAX_LINEAR;
    const angular = rightStick.getValue() * MAX_ANGULAR;
    RamblaWS.sendCmdVel(linear, angular);
  }, CMD_VEL_RATE_MS);
}
