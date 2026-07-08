// Camera feed is a plain <img src="/camera/stream"> MJPEG stream (see
// server.py) - no JS decode/render logic needed. This file only handles
// the "feed looks frozen" edge case: if the <img> errors (e.g. backend
// restarted), retry by re-setting src after a short delay.
function initCameraFeed() {
  const img = document.getElementById('camera-feed');
  img.addEventListener('error', () => {
    setTimeout(() => {
      img.src = `/camera/stream?retry=${Date.now()}`;
    }, 1000);
  });
}
