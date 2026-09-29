// Limelight mount comparison: runs the coverage model from limelight-coverage.html for a set of mounts and prints a
// markdown table. Usage: node tools/limelight-mount-table.js [lensHeightIn] > doc/limelight-mount.md
const fs = require('fs');
const path = require('path');
const html = fs.readFileSync(path.join(__dirname, 'limelight-coverage.html'), 'utf8');
const model = html.split('<script>')[1].split('// ---- drawing')[0].replace(/^const /gm, 'var '); // top-level consts would stay local to eval
eval(model);

const height = +(process.argv[2] || 11);
const tilts = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45];
const range = (a) => (a.length ? `${a[0]}–${a[a.length - 1]}` : 'none');

function measure(orient, tilt) {
  S.orient = orient; S.p = tilt; S.h = height; S.alliance = 'Red'; S.up = 'South';
  // 1. straight out from the up cell along its axis
  const op = opening(S.up);
  const up = [], dn = [], any = [];
  for (let d = 0; d <= 110; d++) {
    const e = evaluate(op.x, op.y + d);
    if (e.upSeen) up.push(d); else if (e.dnSeen) dn.push(d);
    if (e.upSeen || e.dnSeen) any.push(d);
  }
  // gaps: distances between 0 and the far end where nothing is read
  const far = any.length ? any[any.length - 1] : 0;
  const gaps = any.length ? far + 1 - any.length - any[0] : 0;
  // 2. whole field: every 2 in grid point a robot can reach (outside the frame's feet), turret aimed at the up cell
  let cells = 0, readable = 0, band = 0, bandUp = 0;
  for (let x = -63; x <= 63; x += 2) for (let y = -63; y <= 63; y += 2) {
    if (Math.abs(Math.abs(x) - H.frameW / 2) < 10 && Math.abs(y) < H.frameD / 2 + 9) continue; // robot body over a foot bar
    const e = evaluate(x, y);
    cells++;
    if (e.upSeen || e.dnSeen) readable++;
    const dOpen = Math.hypot(x - op.x, y - op.y);
    if (dOpen >= 30 && dOpen <= 66 && Math.sign(y - op.y) === 1) { // usual shots: 30-66 in, on the cell's outward side
      band++;
      if (e.upSeen) bandUp++;
    }
  }
  return { up: range(up), dn: range(dn), gaps, field: readable / cells, shots: bandUp / band };
}

const pct = (v) => `${Math.round(v * 100)}%`;
const lines = [];
lines.push(`# Limelight 3A coverage by mount, lens ${height} in above the floor`);
lines.push('');
lines.push('| Camera orientation | Camera tilt above horizontal | Distances straight out from the up cell\'s opening where the up cell\'s tags are read (in) | Distances straight out where only the down cell\'s tags are read (in) | Inches along that line where no tags are read | Share of the field where the hive state can be read | Share of shooting spots (30–66 in from the opening) that see the up cell\'s tags |');
lines.push('|---|---|---|---|---|---|---|');
for (const orient of ['landscape', 'portrait']) for (const t of tilts) {
  const m = measure(orient, t);
  lines.push(`| ${orient} | ${t}° | ${m.up} | ${m.dn} | ${m.gaps} | ${pct(m.field)} | ${pct(m.shots)} |`);
}
console.log(lines.join('\n'));
