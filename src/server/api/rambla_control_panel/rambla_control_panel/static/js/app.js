const MOBILE_BREAKPOINT_PX = 767;

function initTabs() {
  const buttons = document.querySelectorAll('.tab-button');
  buttons.forEach((btn) => {
    btn.addEventListener('click', () => {
      buttons.forEach((b) => b.classList.remove('active'));
      document.querySelectorAll('.tab-panel').forEach((p) => p.classList.remove('active'));
      btn.classList.add('active');
      document.getElementById(`tab-${btn.dataset.tab}`).classList.add('active');
    });
  });
}

function initConnectionIndicator() {
  const el = document.getElementById('connection-indicator');
  RamblaWS.onConnectionChange((connected) => {
    el.classList.toggle('connected', connected);
    el.classList.toggle('disconnected', !connected);
  });
}

function initViewportClass() {
  function update() {
    document.body.classList.toggle('mobile', window.innerWidth <= MOBILE_BREAKPOINT_PX);
  }
  window.addEventListener('resize', update);
  update();
}

window.addEventListener('DOMContentLoaded', () => {
  initTabs();
  initConnectionIndicator();
  initViewportClass();
  initJoysticks();
  initCameraFeed();
  initDebugTabs();
});
