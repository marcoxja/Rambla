// Camera feed is now binary JPEG frames over the relay's video WebSocket
// (relay-protocol.md "Video-channel framing") instead of an MJPEG
// multipart `<img src="/camera/stream">` stream - the gateway no longer
// runs an HTTP server at all (M4). Each frame is a Blob; render it by
// pointing the <img> at an object URL and revoking the previous one once
// the new frame has loaded, so we never hold more than two URLs live.
function initCameraFeed() {
  const img = document.getElementById('camera-feed');
  let previousUrl = null;

  RamblaWS.onVideoFrame((blob) => {
    const url = URL.createObjectURL(blob);
    const toRevoke = previousUrl;
    img.onload = () => {
      if (toRevoke) URL.revokeObjectURL(toRevoke);
    };
    previousUrl = url;
    img.src = url;
  });
}
