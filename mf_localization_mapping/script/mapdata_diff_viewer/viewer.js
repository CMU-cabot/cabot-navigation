/* global window, document, FileReader */

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function getChecked(id) {
  const el = document.getElementById(id);
  return Boolean(el && el.checked);
}

function getNumber(id) {
  const el = document.getElementById(id);
  return el ? Number(el.value) : 0;
}

function colorForLayer(layer) {
  if (layer === "orig") return "#777";
  if (layer === "shifted") return "#1f77b4";
  if (layer === "delta") return "#ff7f0e";
  return "#999";
}

function isFiniteNumber(x) {
  return typeof x === "number" && Number.isFinite(x);
}

function boundsInit() {
  return { minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity };
}

function boundsExpand(b, x, y) {
  b.minX = Math.min(b.minX, x);
  b.minY = Math.min(b.minY, y);
  b.maxX = Math.max(b.maxX, x);
  b.maxY = Math.max(b.maxY, y);
}

function boundsFromFeatures(features) {
  const b = boundsInit();
  for (const f of features) {
    const g = f.geometry;
    if (!g || typeof g !== "object") continue;
    const t = g.type;
    const c = g.coordinates;
    if (t === "Point" && Array.isArray(c) && c.length >= 2) {
      const x = Number(c[0]);
      const y = Number(c[1]);
      if (isFiniteNumber(x) && isFiniteNumber(y)) boundsExpand(b, x, y);
    } else if (t === "LineString" && Array.isArray(c)) {
      for (const p of c) {
        if (!Array.isArray(p) || p.length < 2) continue;
        const x = Number(p[0]);
        const y = Number(p[1]);
        if (isFiniteNumber(x) && isFiniteNumber(y)) boundsExpand(b, x, y);
      }
    } else if (t === "Polygon" && Array.isArray(c)) {
      for (const ring of c) {
        if (!Array.isArray(ring)) continue;
        for (const p of ring) {
          if (!Array.isArray(p) || p.length < 2) continue;
          const x = Number(p[0]);
          const y = Number(p[1]);
          if (isFiniteNumber(x) && isFiniteNumber(y)) boundsExpand(b, x, y);
        }
      }
    }
  }
  if (!Number.isFinite(b.minX)) return null;
  return b;
}

function fitTransformToBounds(bounds, width, height) {
  const pad = 24;
  const w = Math.max(1, width - pad * 2);
  const h = Math.max(1, height - pad * 2);
  const spanX = Math.max(1e-12, bounds.maxX - bounds.minX);
  const spanY = Math.max(1e-12, bounds.maxY - bounds.minY);
  const s = Math.min(w / spanX, h / spanY);
  const cx = (bounds.minX + bounds.maxX) / 2;
  const cy = (bounds.minY + bounds.maxY) / 2;
  return {
    scale: s,
    offsetX: width / 2 - cx * s,
    offsetY: height / 2 + cy * s, // y inverted
  };
}

function worldToScreen(x, y, transform) {
  return {
    sx: x * transform.scale + transform.offsetX,
    sy: -y * transform.scale + transform.offsetY,
  };
}

function screenToWorld(sx, sy, transform) {
  return {
    x: (sx - transform.offsetX) / transform.scale,
    y: -(sy - transform.offsetY) / transform.scale,
  };
}

function passesFilters(f) {
  const p = f.properties || {};
  const moved = Boolean(p.moved);
  if (moved && !getChecked("showMoved")) return false;
  if (!moved && !getChecked("showNotMoved")) return false;

  const layer = String(p.layer || "");
  if (layer === "orig" && !getChecked("showOrig")) return false;
  if (layer === "shifted" && !getChecked("showShifted")) return false;
  if (layer === "delta" && !getChecked("showDelta")) return false;

  const kind = String(p.kind || "");
  if (kind === "node" && !getChecked("kindNode")) return false;
  if (kind === "link" && !getChecked("kindLink")) return false;
  if (kind === "facility" && !getChecked("kindFacility")) return false;
  if (kind === "poi" && !getChecked("kindPoi")) return false;
  return true;
}

function summarize(features) {
  const counts = new Map();
  for (const f of features) {
    const p = f.properties || {};
    const kind = String(p.kind || "unknown");
    const layer = String(p.layer || "unknown");
    const moved = Boolean(p.moved) ? "moved" : "not_moved";
    const k = `${kind}\t${layer}\t${moved}`;
    counts.set(k, (counts.get(k) || 0) + 1);
  }
  const lines = [];
  for (const [k, v] of Array.from(counts.entries()).sort()) {
    lines.push(`${k}\t${v}`);
  }
  if (lines.length === 0) return "No features.";
  return lines.join("\n");
}

function main() {
  const canvas = document.getElementById("c");
  const ctx = canvas.getContext("2d");
  const summaryEl = document.getElementById("summary");

  let features = [];
  let dataBounds = null;
  let transform = { scale: 1, offsetX: 0, offsetY: 0 };

  function resize() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.floor(rect.width * dpr));
    canvas.height = Math.max(1, Math.floor(rect.height * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    if (dataBounds) transform = fitTransformToBounds(dataBounds, rect.width, rect.height);
    draw();
  }

  function drawPoint(x, y, color, r, alpha) {
    const { sx, sy } = worldToScreen(x, y, transform);
    ctx.globalAlpha = alpha;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, Math.PI * 2);
    ctx.fill();
  }

  function drawLine(coords, color, width, alpha) {
    if (!Array.isArray(coords) || coords.length < 2) return;
    ctx.globalAlpha = alpha;
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    for (let i = 0; i < coords.length; i++) {
      const p = coords[i];
      if (!Array.isArray(p) || p.length < 2) continue;
      const x = Number(p[0]);
      const y = Number(p[1]);
      if (!isFiniteNumber(x) || !isFiniteNumber(y)) continue;
      const { sx, sy } = worldToScreen(x, y, transform);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
    }
    ctx.stroke();
  }

  function draw() {
    const rect = canvas.getBoundingClientRect();
    ctx.clearRect(0, 0, rect.width, rect.height);
    ctx.save();

    const pointSize = getNumber("pointSize");
    const lineWidth = getNumber("lineWidth");
    const opacity = getNumber("opacity");

    // Draw in stable order: delta under points? Actually delta vectors on top.
    const order = { orig: 0, shifted: 1, delta: 2 };
    const filtered = features.filter(passesFilters).sort((a, b) => {
      const la = String((a.properties || {}).layer || "");
      const lb = String((b.properties || {}).layer || "");
      return (order[la] ?? 9) - (order[lb] ?? 9);
    });

    for (const f of filtered) {
      const g = f.geometry;
      if (!g || typeof g !== "object") continue;
      const p = f.properties || {};
      const layer = String(p.layer || "");
      const moved = Boolean(p.moved);
      const alpha = opacity * (moved ? 1.0 : 0.25);
      const color = colorForLayer(layer);

      if (g.type === "Point" && Array.isArray(g.coordinates) && g.coordinates.length >= 2) {
        const x = Number(g.coordinates[0]);
        const y = Number(g.coordinates[1]);
        if (!isFiniteNumber(x) || !isFiniteNumber(y)) continue;
        drawPoint(x, y, color, pointSize, alpha);
      } else if (g.type === "LineString") {
        drawLine(g.coordinates, color, lineWidth, alpha);
      } else if (g.type === "Polygon" && Array.isArray(g.coordinates)) {
        // draw outline only
        for (const ring of g.coordinates) {
          drawLine(ring, color, lineWidth, alpha);
        }
      }
    }

    ctx.restore();
  }

  function loadGeoJSON(text) {
    const obj = JSON.parse(text);
    const feats = obj && obj.features;
    if (!Array.isArray(feats)) throw new Error("Invalid GeoJSON: missing features[]");
    features = feats;
    dataBounds = boundsFromFeatures(features);
    summaryEl.textContent = summarize(features);
    resize();
  }

  const fileInput = document.getElementById("file");
  fileInput.addEventListener("change", () => {
    const f = fileInput.files && fileInput.files[0];
    if (!f) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        loadGeoJSON(String(reader.result));
      } catch (e) {
        summaryEl.textContent = `Error: ${e && e.message ? e.message : String(e)}`;
      }
    };
    reader.readAsText(f);
  });

  // Pan/zoom
  let dragging = false;
  let last = null;
  canvas.addEventListener("mousedown", (ev) => {
    dragging = true;
    canvas.classList.add("dragging");
    last = { x: ev.clientX, y: ev.clientY };
  });
  window.addEventListener("mouseup", () => {
    dragging = false;
    canvas.classList.remove("dragging");
    last = null;
  });
  window.addEventListener("mousemove", (ev) => {
    if (!dragging || !last) return;
    const dx = ev.clientX - last.x;
    const dy = ev.clientY - last.y;
    last = { x: ev.clientX, y: ev.clientY };
    transform.offsetX += dx;
    transform.offsetY += dy;
    draw();
  });
  canvas.addEventListener(
    "wheel",
    (ev) => {
      ev.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const mx = ev.clientX - rect.left;
      const my = ev.clientY - rect.top;
      const before = screenToWorld(mx, my, transform);
      const delta = ev.deltaY;
      const factor = Math.pow(1.2, -delta / 100);
      const newScale = clamp(transform.scale * factor, 1e-6, 1e8);
      transform.scale = newScale;
      const after = worldToScreen(before.x, before.y, transform);
      transform.offsetX += mx - after.sx;
      transform.offsetY += my - after.sy;
      draw();
    },
    { passive: false }
  );

  document.getElementById("fit").addEventListener("click", () => {
    if (!dataBounds) return;
    const rect = canvas.getBoundingClientRect();
    transform = fitTransformToBounds(dataBounds, rect.width, rect.height);
    draw();
  });

  const filterIds = [
    "showMoved",
    "showNotMoved",
    "showOrig",
    "showShifted",
    "showDelta",
    "kindNode",
    "kindLink",
    "kindFacility",
    "kindPoi",
    "pointSize",
    "lineWidth",
    "opacity",
  ];
  for (const id of filterIds) {
    const el = document.getElementById(id);
    if (!el) continue;
    el.addEventListener("input", draw);
    el.addEventListener("change", draw);
  }

  window.addEventListener("resize", resize);
  resize();
}

main();

