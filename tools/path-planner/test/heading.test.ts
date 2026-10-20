import { describe, expect, it } from 'vitest';
import { ArcTable, type Cubic } from '../src/core/bezier.ts';
import { pedroHeading, smoothHeading, toPieces } from '../src/core/heading.ts';

const curve: Cubic = [
  { x: 0, y: 0 },
  { x: 5, y: 0 },
  { x: 60, y: 40 },
  { x: 100, y: 40 },
];

describe('headings', () => {
  it('hits every breakpoint exactly, the way Pedro evaluates them', () => {
    const table = new ArcTable(curve, 256);
    const pieces = { F: [0, 0.25, 0.6, 1], H: [0, 0.4, 1.2, 1.5] };
    pieces.F.forEach((f, k) => {
      expect(pedroHeading(table, pieces, table.parameter(f))).toBeCloseTo(pieces.H[k], 3);
    });
  });

  it('stays continuous across breakpoints', () => {
    const table = new ArcTable(curve, 256);
    const pieces = { F: [0, 0.5, 1], H: [0, 1, 0.5] };
    const t = table.parameter(0.5);
    expect(pedroHeading(table, pieces, t - 1e-6)).toBeCloseTo(pedroHeading(table, pieces, t + 1e-6), 3);
  });

  it('samples a smooth heading into pieces that follow it closely', () => {
    const h = smoothHeading([0, 50, 100], [0, 1, 3]);
    const pieces = toPieces(h, 0, 100, 4);
    for (let i = 0; i < pieces.F.length; i++) expect(pieces.H[i]).toBeCloseTo(h(pieces.F[i] * 100), 9);
    for (let i = 1; i < pieces.H.length; i++) expect(Math.abs(pieces.H[i] - pieces.H[i - 1])).toBeLessThan(Math.PI);
  });

  it('never overshoots between knots', () => {
    const h = smoothHeading([0, 10, 20, 30], [0, 1, 1, 0]);
    for (let s = 0; s <= 30; s += 0.5) {
      expect(h(s)).toBeLessThanOrEqual(1 + 1e-9);
      expect(h(s)).toBeGreaterThanOrEqual(-1e-9);
    }
  });
});
