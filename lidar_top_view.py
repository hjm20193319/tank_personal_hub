"""
lidar_top_view.py

테스트 전용 LiDAR 실시간 top view 시각화 모듈이다.
/info EndPoint가 메모리에 저장한 isDetected=True LiDAR 포인트 전체를
브라우저 Canvas에서 x-z 평면으로 표시한다.

삭제 방법:
1. tank_sensor.py의 lidar_top_view_blueprint import를 제거한다.
2. app.register_blueprint(lidar_top_view_blueprint)를 제거한다.
3. 이 파일을 삭제한다.
"""

import time

from flask import Blueprint, Response, jsonify

from lidar_endpoint_store import get_latest_lidar_points


lidar_top_view_blueprint = Blueprint("lidar_top_view", __name__)


@lidar_top_view_blueprint.get("/lidar-top-view")
def lidar_top_view():
    return Response(LIDAR_TOP_VIEW_HTML, mimetype="text/html")


@lidar_top_view_blueprint.get("/lidar-top-view/data")
def lidar_top_view_data():
    points = get_latest_lidar_points()
    return jsonify(
        {
            "timestamp": time.time(),
            "pointCount": len(points),
            "points": points,
        }
    )


LIDAR_TOP_VIEW_HTML = r"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LiDAR Top View</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b0d0f;
      --surface: #111417;
      --surface-strong: #171b1f;
      --border: #30363c;
      --grid: #252b30;
      --axis: #68727b;
      --text: #f0f3f5;
      --muted: #9ba5ad;
      --cyan: #23d8e5;
      --green: #54d77a;
      --amber: #f4b000;
      --danger: #ff6b6b;
    }

    * { box-sizing: border-box; }

    html, body {
      width: 100%;
      height: 100%;
      margin: 0;
      overflow: hidden;
      background: var(--bg);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }

    body { min-width: 760px; }

    .app {
      display: grid;
      grid-template-rows: 72px minmax(0, 1fr) 42px;
      width: 100%;
      height: 100%;
    }

    .toolbar {
      display: flex;
      align-items: center;
      gap: 24px;
      padding: 0 24px;
      border-bottom: 1px solid var(--border);
      background: var(--surface);
    }

    h1 {
      margin: 0;
      padding-right: 24px;
      border-right: 1px solid var(--border);
      font-size: 22px;
      line-height: 1;
      letter-spacing: -0.02em;
      white-space: nowrap;
    }

    .metric {
      display: flex;
      align-items: baseline;
      gap: 8px;
      color: var(--muted);
      font-size: 13px;
      white-space: nowrap;
    }

    .metric strong {
      color: var(--cyan);
      font-size: 16px;
      font-variant-numeric: tabular-nums;
      font-weight: 650;
    }

    .status-dot {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      background: var(--danger);
      box-shadow: 0 0 0 3px rgba(255, 107, 107, 0.12);
    }

    .status-dot.live {
      background: var(--green);
      box-shadow: 0 0 0 3px rgba(84, 215, 122, 0.12);
    }

    .status-value { color: var(--text); font-weight: 650; }
    .toolbar-spacer { flex: 1; }

    label {
      display: flex;
      align-items: center;
      gap: 10px;
      color: var(--muted);
      font-size: 13px;
      white-space: nowrap;
    }

    select, button {
      height: 38px;
      border: 1px solid var(--border);
      border-radius: 5px;
      background: var(--surface-strong);
      color: var(--text);
      font: 600 13px/1 Inter, ui-sans-serif, system-ui, sans-serif;
    }

    select { min-width: 112px; padding: 0 34px 0 12px; }

    button {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      min-width: 108px;
      padding: 0 14px;
      border-color: #8a6500;
      color: #ffc52d;
      cursor: pointer;
    }

    button:hover { background: #201b0d; border-color: var(--amber); }
    button:focus-visible, select:focus-visible { outline: 2px solid var(--cyan); outline-offset: 2px; }
    button svg { width: 16px; height: 16px; stroke: currentColor; }

    .stage {
      position: relative;
      min-height: 0;
      overflow: hidden;
      background: var(--bg);
    }

    canvas {
      display: block;
      width: 100%;
      height: 100%;
      cursor: crosshair;
    }

    canvas.dragging { cursor: grabbing; }

    .legend {
      position: absolute;
      right: 20px;
      bottom: 18px;
      width: 206px;
      padding: 12px 14px;
      border: 1px solid var(--border);
      border-radius: 5px;
      background: rgba(17, 20, 23, 0.92);
      pointer-events: none;
    }

    .legend-title {
      margin-bottom: 9px;
      color: var(--text);
      font-size: 12px;
      font-weight: 650;
    }

    .legend-ramp {
      height: 8px;
      background: linear-gradient(90deg, #2675ff, #23d8e5, #54d77a, #ffe04b, #ff8a1f);
    }

    .legend-labels {
      display: flex;
      justify-content: space-between;
      margin-top: 6px;
      color: var(--muted);
      font-size: 10px;
      font-variant-numeric: tabular-nums;
    }

    .tooltip {
      position: absolute;
      display: none;
      min-width: 142px;
      padding: 10px 11px;
      border: 1px solid #69737b;
      border-radius: 4px;
      background: rgba(11, 13, 15, 0.96);
      color: var(--text);
      font-size: 12px;
      line-height: 1.55;
      pointer-events: none;
      font-variant-numeric: tabular-nums;
    }

    .empty {
      position: absolute;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      color: var(--muted);
      font-size: 14px;
      pointer-events: none;
    }

    .statusbar {
      display: flex;
      align-items: center;
      gap: 20px;
      padding: 0 24px;
      border-top: 1px solid var(--border);
      background: var(--surface);
      color: var(--muted);
      font-size: 12px;
    }

    .statusbar span + span {
      padding-left: 20px;
      border-left: 1px solid var(--border);
    }

    .disposable { margin-left: auto; color: var(--amber); }

    @media (max-width: 1050px) {
      .refresh-metric, .statusbar .hint { display: none; }
      .toolbar { gap: 14px; padding: 0 16px; }
      h1 { padding-right: 14px; }
    }
  </style>
</head>
<body>
  <main class="app">
    <header class="toolbar">
      <h1>LiDAR Top View</h1>
      <div class="metric">
        <span id="statusDot" class="status-dot"></span>
        <span>Status</span>
        <span id="statusText" class="status-value">WAITING</span>
      </div>
      <div class="metric"><span>Points</span><strong id="pointCount">0</strong></div>
      <div class="metric refresh-metric"><span>Refresh</span><strong id="refreshRate">0.0 Hz</strong></div>
      <div class="toolbar-spacer"></div>
      <label>
        Color
        <select id="colorMode" aria-label="포인트 색상 기준">
          <option value="height">Height</option>
          <option value="distance">Distance</option>
        </select>
      </label>
      <button id="pauseButton" type="button">
        <svg viewBox="0 0 24 24" fill="none" stroke-width="2"><path d="M8 5v14M16 5v14"/></svg>
        <span>Pause</span>
      </button>
      <button id="fitButton" type="button">
        <svg viewBox="0 0 24 24" fill="none" stroke-width="2"><path d="M8 3H3v5M16 3h5v5M8 21H3v-5M16 21h5v-5"/></svg>
        <span>Fit View</span>
      </button>
    </header>

    <section id="stage" class="stage" aria-label="LiDAR x-z top view canvas">
      <canvas id="canvas"></canvas>
      <div id="emptyState" class="empty">/info에서 감지된 LiDAR 포인트를 기다리는 중입니다.</div>
      <div class="legend">
        <div id="legendTitle" class="legend-title">Height (m)</div>
        <div class="legend-ramp"></div>
        <div class="legend-labels"><span id="legendMin">0.0</span><span id="legendMid">0.0</span><span id="legendMax">0.0</span></div>
      </div>
      <div id="tooltip" class="tooltip"></div>
    </section>

    <footer class="statusbar">
      <span>Source: /info → memory snapshot</span>
      <span id="boundsText">Bounds: —</span>
      <span class="hint">Wheel: zoom · Drag: pan · Hover: inspect</span>
      <span class="disposable">테스트 전용 · 삭제 가능한 모듈</span>
    </footer>
  </main>

  <script>
    const canvas = document.getElementById('canvas');
    const context = canvas.getContext('2d', { alpha: false });
    const stage = document.getElementById('stage');
    const tooltip = document.getElementById('tooltip');
    const emptyState = document.getElementById('emptyState');
    const pointCount = document.getElementById('pointCount');
    const refreshRate = document.getElementById('refreshRate');
    const statusDot = document.getElementById('statusDot');
    const statusText = document.getElementById('statusText');
    const boundsText = document.getElementById('boundsText');
    const colorMode = document.getElementById('colorMode');
    const pauseButton = document.getElementById('pauseButton');
    const fitButton = document.getElementById('fitButton');
    const legendTitle = document.getElementById('legendTitle');
    const legendMin = document.getElementById('legendMin');
    const legendMid = document.getElementById('legendMid');
    const legendMax = document.getElementById('legendMax');

    let points = [];
    let paused = false;
    let fitted = false;
    let scale = 10;
    let offsetX = 0;
    let offsetZ = 0;
    let valueMin = 0;
    let valueMax = 1;
    let lastFetchAt = 0;
    let dragStart = null;
    let hoverFrame = null;

    function resizeCanvas() {
      const ratio = window.devicePixelRatio || 1;
      const rect = stage.getBoundingClientRect();
      canvas.width = Math.max(1, Math.round(rect.width * ratio));
      canvas.height = Math.max(1, Math.round(rect.height * ratio));
      canvas.style.width = `${rect.width}px`;
      canvas.style.height = `${rect.height}px`;
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      if (!fitted && points.length) fitView();
      draw();
    }

    function pointValue(point) {
      return colorMode.value === 'distance' ? point.distance : point.y;
    }

    function calculateValueRange() {
      if (!points.length) {
        valueMin = 0;
        valueMax = 1;
        return;
      }
      const values = points.map(pointValue).filter(Number.isFinite).sort((a, b) => a - b);
      const lowIndex = Math.floor((values.length - 1) * 0.02);
      const highIndex = Math.floor((values.length - 1) * 0.98);
      valueMin = values[lowIndex] ?? 0;
      valueMax = values[highIndex] ?? valueMin + 1;
      if (valueMax <= valueMin) valueMax = valueMin + 1;
      legendTitle.textContent = colorMode.value === 'distance' ? 'Distance (m)' : 'Height (m)';
      legendMin.textContent = valueMin.toFixed(1);
      legendMid.textContent = ((valueMin + valueMax) / 2).toFixed(1);
      legendMax.textContent = valueMax.toFixed(1);
    }

    function colorFor(point) {
      const normalized = Math.max(0, Math.min(1, (pointValue(point) - valueMin) / (valueMax - valueMin)));
      const stops = [
        [38, 117, 255],
        [35, 216, 229],
        [84, 215, 122],
        [255, 224, 75],
        [255, 138, 31],
      ];
      const scaled = normalized * (stops.length - 1);
      const index = Math.min(stops.length - 2, Math.floor(scaled));
      const mix = scaled - index;
      const start = stops[index];
      const end = stops[index + 1];
      const channel = i => Math.round(start[i] + (end[i] - start[i]) * mix);
      return `rgb(${channel(0)}, ${channel(1)}, ${channel(2)})`;
    }

    function worldToScreen(x, z) {
      return {
        x: canvas.clientWidth / 2 + (x - offsetX) * scale,
        y: canvas.clientHeight / 2 - (z - offsetZ) * scale,
      };
    }

    function screenToWorld(x, y) {
      return {
        x: (x - canvas.clientWidth / 2) / scale + offsetX,
        z: (canvas.clientHeight / 2 - y) / scale + offsetZ,
      };
    }

    function niceStep(rawStep) {
      const magnitude = 10 ** Math.floor(Math.log10(Math.max(rawStep, 0.0001)));
      const normalized = rawStep / magnitude;
      const nice = normalized < 2 ? 1 : normalized < 5 ? 2 : normalized < 10 ? 5 : 10;
      return nice * magnitude;
    }

    function drawGrid() {
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;
      const topLeft = screenToWorld(0, 0);
      const bottomRight = screenToWorld(width, height);
      const step = niceStep(90 / scale);

      context.lineWidth = 1;
      context.font = '11px Inter, system-ui, sans-serif';
      context.fillStyle = '#7f8991';

      const startX = Math.floor(topLeft.x / step) * step;
      for (let x = startX; x <= bottomRight.x; x += step) {
        const screen = worldToScreen(x, 0);
        context.strokeStyle = Math.abs(x) < step / 100 ? '#68727b' : '#252b30';
        context.beginPath();
        context.moveTo(screen.x, 0);
        context.lineTo(screen.x, height);
        context.stroke();
        context.fillText(x.toFixed(step < 1 ? 1 : 0), screen.x + 4, height - 8);
      }

      const startZ = Math.floor(bottomRight.z / step) * step;
      for (let z = startZ; z <= topLeft.z; z += step) {
        const screen = worldToScreen(0, z);
        context.strokeStyle = Math.abs(z) < step / 100 ? '#68727b' : '#252b30';
        context.beginPath();
        context.moveTo(0, screen.y);
        context.lineTo(width, screen.y);
        context.stroke();
        context.fillText(z.toFixed(step < 1 ? 1 : 0), 7, screen.y - 5);
      }

      context.fillStyle = '#aab2b8';
      context.fillText('+X', width - 28, height / 2 - 8);
      context.fillText('+Z', width / 2 + 8, 16);
    }

    function draw() {
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;
      context.fillStyle = '#0b0d0f';
      context.fillRect(0, 0, width, height);
      drawGrid();

      const radius = Math.max(1, Math.min(2.3, scale / 18));
      for (const point of points) {
        const screen = worldToScreen(point.x, point.z);
        if (screen.x < -3 || screen.x > width + 3 || screen.y < -3 || screen.y > height + 3) continue;
        context.fillStyle = colorFor(point);
        context.fillRect(screen.x - radius / 2, screen.y - radius / 2, radius, radius);
      }

      const origin = worldToScreen(0, 0);
      if (origin.x >= 0 && origin.x <= width && origin.y >= 0 && origin.y <= height) {
        context.strokeStyle = '#f0f3f5';
        context.lineWidth = 1.5;
        context.beginPath();
        context.moveTo(origin.x - 7, origin.y);
        context.lineTo(origin.x + 7, origin.y);
        context.moveTo(origin.x, origin.y - 7);
        context.lineTo(origin.x, origin.y + 7);
        context.stroke();
      }
    }

    function fitView() {
      if (!points.length) return;
      const xs = points.map(point => point.x);
      const zs = points.map(point => point.z);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minZ = Math.min(...zs);
      const maxZ = Math.max(...zs);
      offsetX = (minX + maxX) / 2;
      offsetZ = (minZ + maxZ) / 2;
      const spanX = Math.max(1, maxX - minX);
      const spanZ = Math.max(1, maxZ - minZ);
      scale = Math.max(0.05, Math.min((canvas.clientWidth - 80) / spanX, (canvas.clientHeight - 80) / spanZ));
      fitted = true;
      boundsText.textContent = `Bounds: X ${minX.toFixed(1)}…${maxX.toFixed(1)} · Z ${minZ.toFixed(1)}…${maxZ.toFixed(1)}`;
      draw();
    }

    async function refresh() {
      if (paused) return;
      try {
        const startedAt = performance.now();
        const response = await fetch('/lidar-top-view/data', { cache: 'no-store' });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const payload = await response.json();
        points = payload.points || [];
        pointCount.textContent = Number(payload.pointCount || 0).toLocaleString();
        emptyState.style.display = points.length ? 'none' : 'flex';
        statusDot.classList.add('live');
        statusText.textContent = 'LIVE';
        const now = performance.now();
        if (lastFetchAt) refreshRate.textContent = `${(1000 / (now - lastFetchAt)).toFixed(1)} Hz`;
        lastFetchAt = now;
        calculateValueRange();
        if (!fitted && points.length) fitView(); else draw();
        void startedAt;
      } catch (error) {
        statusDot.classList.remove('live');
        statusText.textContent = 'OFFLINE';
      }
    }

    pauseButton.addEventListener('click', () => {
      paused = !paused;
      pauseButton.querySelector('span').textContent = paused ? 'Resume' : 'Pause';
      statusText.textContent = paused ? 'PAUSED' : 'LIVE';
      if (!paused) refresh();
    });

    fitButton.addEventListener('click', fitView);

    colorMode.addEventListener('change', () => {
      calculateValueRange();
      draw();
    });

    canvas.addEventListener('wheel', event => {
      event.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const mouseX = event.clientX - rect.left;
      const mouseY = event.clientY - rect.top;
      const before = screenToWorld(mouseX, mouseY);
      scale = Math.max(0.02, Math.min(500, scale * (event.deltaY < 0 ? 1.12 : 0.89)));
      const after = screenToWorld(mouseX, mouseY);
      offsetX += before.x - after.x;
      offsetZ += before.z - after.z;
      draw();
    }, { passive: false });

    canvas.addEventListener('pointerdown', event => {
      dragStart = { x: event.clientX, y: event.clientY, offsetX, offsetZ };
      canvas.classList.add('dragging');
      canvas.setPointerCapture(event.pointerId);
    });

    canvas.addEventListener('pointermove', event => {
      if (dragStart) {
        offsetX = dragStart.offsetX - (event.clientX - dragStart.x) / scale;
        offsetZ = dragStart.offsetZ + (event.clientY - dragStart.y) / scale;
        draw();
        return;
      }
      if (hoverFrame) cancelAnimationFrame(hoverFrame);
      hoverFrame = requestAnimationFrame(() => showTooltip(event));
    });

    canvas.addEventListener('pointerup', event => {
      dragStart = null;
      canvas.classList.remove('dragging');
      canvas.releasePointerCapture(event.pointerId);
    });

    canvas.addEventListener('pointerleave', () => {
      tooltip.style.display = 'none';
    });

    function showTooltip(event) {
      if (!points.length) return;
      const rect = canvas.getBoundingClientRect();
      const mouseX = event.clientX - rect.left;
      const mouseY = event.clientY - rect.top;
      let nearest = null;
      let nearestDistance = 10 * 10;
      for (const point of points) {
        const screen = worldToScreen(point.x, point.z);
        const dx = screen.x - mouseX;
        const dy = screen.y - mouseY;
        const distanceSquared = dx * dx + dy * dy;
        if (distanceSquared < nearestDistance) {
          nearest = point;
          nearestDistance = distanceSquared;
        }
      }
      if (!nearest) {
        tooltip.style.display = 'none';
        return;
      }
      tooltip.innerHTML = `X: ${nearest.x.toFixed(2)} m<br>Z: ${nearest.z.toFixed(2)} m<br>Height: ${nearest.y.toFixed(2)} m<br>Distance: ${nearest.distance.toFixed(2)} m<br>Angle: ${nearest.angle.toFixed(2)}°`;
      tooltip.style.display = 'block';
      tooltip.style.left = `${Math.min(stage.clientWidth - 165, mouseX + 15)}px`;
      tooltip.style.top = `${Math.max(12, mouseY - 96)}px`;
    }

    new ResizeObserver(resizeCanvas).observe(stage);
    setInterval(refresh, 200);
    refresh();
  </script>
</body>
</html>
"""