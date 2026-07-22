// Keyboard drive: WASD/arrows -> the same setKeyVelocity() feed joystick.js
// mixes into its existing controlEnabled-gated 20Hz send loop. This file has
// no send path of its own - lease gate, deadman, and rate all come from
// joystick.js for free.
const LINEAR_KEYS = { KeyW: 1, ArrowUp: 1, KeyS: -1, ArrowDown: -1 };
const ANGULAR_KEYS = { KeyA: -1, ArrowLeft: -1, KeyD: 1, ArrowRight: 1 };

const pressedKeys = new Set();
// Keys still physically held when a reset happens (Space, lease loss,
// blur/hide). The OS keeps sending keydown auto-repeat for a held key with
// no intervening keyup, so without this a key held through e.g. a lease
// loss -> regrant would silently resume driving on the next repeat. Held
// here until an actual keyup is observed, forcing a real release+press.
const staleKeys = new Set();

function isTextInputFocused() {
  const el = document.activeElement;
  return !!el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.isContentEditable);
}

function recomputeKeyVelocity() {
  let linear = 0;
  let angular = 0;
  pressedKeys.forEach((code) => {
    if (code in LINEAR_KEYS) linear += LINEAR_KEYS[code];
    if (code in ANGULAR_KEYS) angular += ANGULAR_KEYS[code];
  });
  linear = Math.max(-1, Math.min(1, linear));
  angular = Math.max(-1, Math.min(1, angular));
  setKeyVelocity(linear * MAX_LINEAR, angular * MAX_ANGULAR);
}

// Exported for joystick.js to call on every controlEnabled transition, and
// used locally for the Space e-stop.
function resetKeyboardState() {
  pressedKeys.forEach((code) => staleKeys.add(code));
  pressedKeys.clear();
  setKeyVelocity(0, 0);
}

function initKeyboardControl() {
  document.addEventListener('keydown', (e) => {
    if (isTextInputFocused() || !controlEnabled) return;
    if (e.code === 'Space') {
      resetKeyboardState();
      e.preventDefault();
      return;
    }
    if (staleKeys.has(e.code)) {
      e.preventDefault();
      return;
    }
    if (!(e.code in LINEAR_KEYS) && !(e.code in ANGULAR_KEYS)) return;
    pressedKeys.add(e.code);
    recomputeKeyVelocity();
    e.preventDefault();
  });

  document.addEventListener('keyup', (e) => {
    staleKeys.delete(e.code);
    if (!pressedKeys.delete(e.code)) return;
    recomputeKeyVelocity();
  });

  // Losing focus/visibility mid-keypress must not leave a key "stuck" -
  // there's no matching keyup to clear it once focus returns elsewhere.
  window.addEventListener('blur', resetKeyboardState);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) resetKeyboardState();
  });
}
