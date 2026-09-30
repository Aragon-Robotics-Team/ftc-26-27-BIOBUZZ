// Camera coverage model shared by tools/camera-coverage-3d.html (browser) and tools/limelight-mount-table.js (node).
// Units: inches and degrees. World frame: x from red to blue, y toward the audience, h up, origin at field center.
// Hive geometry from the Competition Manual (Figs 9-9 to 9-17) as built in BiobuzzSim; tag cluster position from
// Fig 9-15 (cluster center 7.19 in behind the opening, i.e. 14.27 in from the pivot along the arm).
(function (root) {
  const HIVE = {
    pivot: 43.95, cx: 12.75, tilt: 30,
    rise: 5.75,            // cell center above the arm axis
    gap: 18.84,            // between the two cells along the arm
    cellD: 12, cellW: 20, cellH: 14, cellSide: 7.61,
    tagAlong: 14.27,       // tag cluster center, distance from the pivot along the arm
    tagUp: 5.75 - 7 - 0.3, // underside of the cell floor, relative to the arm axis
    tagX: [-6.5, -2.75, 2.75, 6.5], tagSize: 3.25,
    frameW: 49.46, frameD: 38.95,
  };
  const FIELD = 144;
  const IDS = { Red: { North: [30, 31, 32, 33], South: [34, 35, 36, 37] }, Blue: { South: [38, 39, 40, 41], North: [42, 43, 44, 45] } };
  const FOV = { limelight: { landscape: [54.5, 42], portrait: [42, 54.5] }, c270: [45, 34] }; // C270 at 640x480: estimate
  const rad = (d) => (d * Math.PI) / 180, deg = (r) => (r * 180) / Math.PI;
  const sub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y, h: a.h - b.h });
  const dot = (a, b) => a.x * b.x + a.y * b.y + a.h * b.h;
  const len = (a) => Math.sqrt(dot(a, a));
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

  // ---- hive. Hub-local frame: x (red to blue), up (perpendicular to the arm), along (+ toward the audience arm).
  const theta = (up) => (up === 'South' ? -rad(HIVE.tilt) : rad(HIVE.tilt)); // up = which cell faces up
  function rot(l, up) {
    const t = theta(up), c = Math.cos(t), s = Math.sin(t);
    return { x: l.x, y: l.up * s + l.along * c, h: l.up * c - l.along * s };
  }
  function hubToWorld(l, alliance, up) {
    const r = rot(l, up);
    return { x: (alliance === 'Red' ? -1 : 1) * HIVE.cx + r.x, y: r.y, h: HIVE.pivot + r.h };
  }
  const sgn = (cell) => (cell === 'South' ? 1 : -1);
  const otherCell = (cell) => (cell === 'South' ? 'North' : 'South');
  function tags(alliance, up, cell) {
    const s = sgn(cell), n = rot({ x: 0, up: -1, along: 0 }, up);
    return HIVE.tagX.map((tx, i) => ({
      id: IDS[alliance][cell][i], cell, n,
      p: hubToWorld({ x: tx * s, up: HIVE.tagUp, along: s * HIVE.tagAlong }, alliance, up),
    }));
  }
  function opening(alliance, up, cell) {
    return hubToWorld({ x: 0, up: HIVE.rise, along: sgn(cell) * (HIVE.gap / 2 + HIVE.cellD) }, alliance, up);
  }
  // pentagon cross-section of a cell (x, up), floor at the bottom, peaked roof on top
  function cellOutline(alliance, up, cell) {
    const W = HIVE.cellW / 2, b = HIVE.rise - HIVE.cellH / 2, s = sgn(cell);
    const pent = [[-W, b], [W, b], [W, b + HIVE.cellSide], [0, b + HIVE.cellH], [-W, b + HIVE.cellSide]];
    const ring = (along) => pent.map(([x, u]) => hubToWorld({ x: x * s, up: u, along: s * along }, alliance, up));
    return { inner: ring(HIVE.gap / 2), outer: ring(HIVE.gap / 2 + HIVE.cellD) };
  }
  function clusterCenter(ts) {
    const c = { x: 0, y: 0, h: 0 };
    for (const t of ts) { c.x += t.p.x / ts.length; c.y += t.p.y / ts.length; c.h += t.p.h / ts.length; }
    return c;
  }

  // ---- cameras
  function camera(C, yaw, pitchDeg, fovDeg) {
    const p = rad(pitchDeg);
    return {
      C, yaw, pitch: p,
      f: { x: Math.cos(p) * Math.cos(yaw), y: Math.cos(p) * Math.sin(yaw), h: Math.sin(p) },
      r: { x: -Math.sin(yaw), y: Math.cos(yaw), h: 0 },
      u: { x: -Math.sin(p) * Math.cos(yaw), y: -Math.sin(p) * Math.sin(yaw), h: Math.cos(p) },
      hf: rad(fovDeg[0]) / 2, vf: rad(fovDeg[1]) / 2,
    };
  }
  function inView(cam, P, marginRad) {
    const d = sub(P, cam.C), zc = dot(d, cam.f);
    if (zc <= 0) return false;
    const m = marginRad || 0;
    return Math.atan(Math.abs(dot(d, cam.r)) / zc) <= cam.hf - m && Math.atan(Math.abs(dot(d, cam.u)) / zc) <= cam.vf - m;
  }
  // limits: range (in, face-on) and off (max degrees off the tag's face). Tags are opaque stickers: back side never reads.
  function readTag(cam, t, limits) {
    const d = sub(t.p, cam.C), dist = len(d);
    const cosOff = -dot(t.n, d) / dist, off = deg(Math.acos(Math.max(-1, Math.min(1, cosOff))));
    const visible = inView(cam, t.p, Math.atan(HIVE.tagSize / 2 / dist));
    const effDist = cosOff > 0.05 ? dist / Math.sqrt(cosOff) : Infinity; // a tilted tag looks smaller one way
    return { dist, off, inView: visible, ok: visible && off <= limits.off && effDist <= limits.range };
  }
  // ray from the camera through normalized image point (a, b) in [-1, 1]
  function ray(cam, a, b) {
    const th = Math.tan(cam.hf), tv = Math.tan(cam.vf);
    return {
      x: cam.f.x + a * th * cam.r.x + b * tv * cam.u.x,
      y: cam.f.y + a * th * cam.r.y + b * tv * cam.u.y,
      h: cam.f.h + a * th * cam.r.h + b * tv * cam.u.h,
    };
  }
  // where the image border meets the floor, clipped to the field
  function floorPolygon(cam, far) {
    const edge = [], N = 24, FAR = far || 300;
    for (let i = 0; i <= N; i++) edge.push([-1 + (2 * i) / N, -1]);
    for (let i = 0; i <= N; i++) edge.push([1, -1 + (2 * i) / N]);
    for (let i = 0; i <= N; i++) edge.push([1 - (2 * i) / N, 1]);
    for (let i = 0; i <= N; i++) edge.push([-1, 1 - (2 * i) / N]);
    let pts = edge.map(([a, b]) => {
      const d = ray(cam, a, b), hl = Math.hypot(d.x, d.y);
      let t = d.h < -1e-6 ? -cam.C.h / d.h : Infinity;
      if (!isFinite(t) || t * hl > FAR) t = FAR / hl;
      return { x: cam.C.x + d.x * t, y: cam.C.y + d.y * t };
    });
    const half = FIELD / 2;
    for (const [k, s] of [['x', 1], ['x', -1], ['y', 1], ['y', -1]]) { // Sutherland-Hodgman against each wall
      const inside = (p) => s * p[k] <= half;
      const cut = (a, b) => { const t = (s * half - a[k]) / (b[k] - a[k]); return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }; };
      const out = [];
      for (let i = 0; i < pts.length; i++) {
        const a = pts[i], b = pts[(i + 1) % pts.length];
        if (inside(a)) { out.push(a); if (!inside(b)) out.push(cut(a, b)); } else if (inside(b)) out.push(cut(a, b));
      }
      pts = out;
      if (!pts.length) break;
    }
    return pts;
  }

  // ---- robot. cfg: { alliance, up, limelight: {h, tilt, orient, range, off}, webcam: {h, tilt, toe, side, fwd, lat} }
  // heading = chassis yaw (radians, 0 = +x). Turret keeps the Limelight on our up cell's tags within +-90 deg of the chassis
  // (clamp = false aims freely, for the coverage maps).
  function limelight(cfg, rx, ry, heading, clamp) {
    const L = cfg.limelight;
    const up = tags(cfg.alliance, cfg.up, cfg.up), dn = tags(cfg.alliance, cfg.up, otherCell(cfg.up));
    const aim = clusterCenter(up);
    const want = Math.atan2(aim.y - ry, aim.x - rx);
    let yaw = want, clamped = false;
    if (clamp) {
      const rel = wrap(want - heading), lim = Math.PI / 2;
      clamped = Math.abs(rel) > lim;
      yaw = heading + Math.max(-lim, Math.min(lim, rel));
    }
    const cam = camera({ x: rx, y: ry, h: L.h }, yaw, L.tilt, FOV.limelight[L.orient]);
    const ru = up.map((t) => readTag(cam, t, L)), rd = dn.map((t) => readTag(cam, t, L));
    return { cam, up, dn, ru, rd, upSeen: ru.some((r) => r.ok), dnSeen: rd.some((r) => r.ok), clamped, turretRel: wrap(yaw - heading) };
  }
  function webcam(cfg, rx, ry, heading) {
    // seen from above, +y (audience) is down the screen when +x points right, so a positive turn is clockwise:
    // the robot's left is heading - 90 deg, and a left-side camera toes in toward the centre by turning positive
    const W = cfg.webcam, lat = W.side === 'left' ? -1 : 1;
    const c = Math.cos(heading), s = Math.sin(heading);
    const lx = W.fwd, ly = lat * W.lat;
    const C = { x: rx + lx * c - ly * s, y: ry + lx * s + ly * c, h: W.h };
    return camera(C, heading - lat * rad(W.toe), -W.tilt, FOV.c270);
  }
  // floor in front of the intake (robot frame: fwd from the intake lip, lat across a 14 in mouth) seen by the webcam
  function webcamMouth(cfg, lip) {
    const cam = webcam(cfg, 0, 0, 0), zones = [[0, 12], [12, 24], [24, 48]];
    const share = zones.map(([a, b]) => {
      let n = 0, k = 0;
      for (let f = a; f <= b; f++) for (let l = -7; l <= 7; l++) { n++; if (inView(cam, { x: lip + f, y: l, h: 0 })) k++; }
      return k / n;
    });
    let near = null;
    for (let f = 0; f <= 144; f += 0.5) if (inView(cam, { x: lip + f, y: 0, h: 0 })) { near = f; break; }
    return { share, near };
  }

  const api = { HIVE, FIELD, IDS, FOV, rad, deg, hubToWorld, tags, opening, cellOutline, clusterCenter, otherCell, camera, inView, readTag, ray, floorPolygon, limelight, webcam, webcamMouth };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.CameraModel = api;
})(this);
