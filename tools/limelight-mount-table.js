// Limelight mount comparison from the coverage model in tools/camera-model.js; prints a markdown table.
// Usage: node tools/limelight-mount-table.js [lensHeightIn] > doc/limelight-mount.md
const M = require('./camera-model.js');

const height = +(process.argv[2] || 11);
const tilts = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45];
const LIMITS = { range: 110, off: 60 }; // estimates until measured on the practice hive
const range = (a) => (a.length ? `${a[0]}–${a[a.length - 1]}` : 'none');

function measure(orient, tilt) {
  const cfg = { alliance: 'Red', up: 'South', limelight: { h: height, tilt, orient, ...LIMITS } };
  // 1. straight out from the up cell's opening along its axis
  const op = M.opening(cfg.alliance, cfg.up, cfg.up);
  const up = [], dn = [], any = [];
  for (let d = 0; d <= 110; d++) {
    const e = M.limelight(cfg, op.x, op.y + d, 0, false);
    if (e.upSeen) up.push(d); else if (e.dnSeen) dn.push(d);
    if (e.upSeen || e.dnSeen) any.push(d);
  }
  const gaps = any.length ? any[any.length - 1] + 1 - any.length - any[0] : 0;
  // 2. whole field: every 2 in grid point a robot can reach (body clear of the hive's foot bars)
  let cells = 0, readable = 0, band = 0, bandUp = 0;
  for (let x = -63; x <= 63; x += 2) for (let y = -63; y <= 63; y += 2) {
    if (Math.abs(Math.abs(x) - M.HIVE.frameW / 2) < 10 && Math.abs(y) < M.HIVE.frameD / 2 + 9) continue;
    const e = M.limelight(cfg, x, y, 0, false);
    cells++;
    if (e.upSeen || e.dnSeen) readable++;
    const dOpen = Math.hypot(x - op.x, y - op.y);
    if (dOpen >= 30 && dOpen <= 66 && y > op.y) { band++; if (e.upSeen) bandUp++; }
  }
  return { up: range(up), dn: range(dn), gaps, field: readable / cells, shots: bandUp / band };
}

const pct = (v) => `${Math.round(v * 100)}%`;
const lines = [];
lines.push(`# Limelight 3A coverage comparison, height ${height}inch`);
lines.push('');
lines.push('| Camera orientation | Camera tilt above horizontal | Distances straight out from the up cell\'s opening where the up cell\'s tags are read (in) | Distances straight out where only the down cell\'s tags are read (in) | Inches along that line where no tags are read | Share of the field where the hive state can be read | Share of shooting spots (30–66 in from the opening) that see the up cell\'s tags |');
lines.push('|---|---|---|---|---|---|---|');
for (const orient of ['landscape', 'portrait']) for (const t of tilts) {
  const m = measure(orient, t);
  lines.push(`| ${orient} | ${t}° | ${m.up} | ${m.dn} | ${m.gaps} | ${pct(m.field)} | ${pct(m.shots)} |`);
}
console.log(lines.join('\n'));
