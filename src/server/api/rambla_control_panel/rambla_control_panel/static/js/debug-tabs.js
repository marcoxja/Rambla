// Renders sensor/diagnostics websocket messages into the Sensors and
// Nodes tabs. Headline values are flagged with class="headline" so
// layout.css can hide the rest on narrow viewports (CTL-007).
function fmt(n) {
  return typeof n === 'number' ? n.toFixed(3) : String(n);
}

function initDebugTabs() {
  const imuEl = document.getElementById('imu-readout');
  const odomEl = document.getElementById('odom-readout');
  const scanCanvas = document.getElementById('scan-canvas');
  const scanCtx = scanCanvas.getContext('2d');
  const nodeListEl = document.getElementById('node-list');
  const topicTableBody = document.querySelector('#topic-table tbody');

  RamblaWS.on('sensor_imu', (data) => {
    imuEl.innerHTML = `
      <dt class="headline">Angular vel z</dt><dd class="headline">${fmt(data.angular_velocity.z)} rad/s</dd>
      <dt>Orientation w</dt><dd>${fmt(data.orientation.w)}</dd>
      <dt>Linear accel x</dt><dd>${fmt(data.linear_acceleration.x)} m/s²</dd>
      <dt>Linear accel y</dt><dd>${fmt(data.linear_acceleration.y)} m/s²</dd>
    `;
  });

  RamblaWS.on('sensor_odom', (data) => {
    const speed = Math.hypot(data.linear_velocity.x, data.linear_velocity.y);
    odomEl.innerHTML = `
      <dt class="headline">Speed</dt><dd class="headline">${fmt(speed)} m/s</dd>
      <dt>Position x</dt><dd>${fmt(data.position.x)} m</dd>
      <dt>Position y</dt><dd>${fmt(data.position.y)} m</dd>
      <dt>Angular vel z</dt><dd>${fmt(data.angular_velocity_z)} rad/s</dd>
    `;
  });

  RamblaWS.on('sensor_scan', (data) => {
    const w = scanCanvas.width, h = scanCanvas.height;
    scanCtx.clearRect(0, 0, w, h);
    const cx = w / 2, cy = h / 2;
    const maxRange = data.range_max || 5;
    const scale = Math.min(w, h) / 2 / maxRange;

    scanCtx.fillStyle = '#4da3ff';
    data.ranges.forEach((r, i) => {
      if (!isFinite(r) || r <= 0) return;
      const angle = data.angle_min + i * data.angle_increment;
      const x = cx + r * Math.cos(angle) * scale;
      const y = cy - r * Math.sin(angle) * scale;
      scanCtx.fillRect(x - 1.5, y - 1.5, 3, 3);
    });

    scanCtx.fillStyle = '#e8eaed';
    scanCtx.beginPath();
    scanCtx.arc(cx, cy, 4, 0, Math.PI * 2);
    scanCtx.fill();
  });

  RamblaWS.on('diagnostics', (data) => {
    nodeListEl.innerHTML = data.nodes.map((n) => `<li>${n}</li>`).join('');

    topicTableBody.innerHTML = data.topics
      .sort((a, b) => a.topic.localeCompare(b.topic))
      .map((t) => {
        const stale = t.age_s === null || t.age_s > 2.0;
        const ageText = t.age_s === null ? 'never' : `${t.age_s.toFixed(1)}s ago`;
        return `<tr><td>${t.topic}</td><td class="${stale ? 'stale' : ''}">${ageText}</td></tr>`;
      })
      .join('');
  });
}
