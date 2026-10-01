import { describe, expect, it } from 'vitest';
import { rect, segmentHitsConvex, vec, type Vec } from '../src/core/geom.ts';
import { routeWorld, seedRoutes } from '../src/core/routes.ts';

describe('starting routes', () => {
  it('goes straight when nothing is in the way', () => {
    const w = routeWorld([], 7, 2, 1);
    expect(seedRoutes(w, vec(20, 20), vec(50, 60))).toEqual([[]]);
  });

  it('finds both ways around a block, shortest first', () => {
    const w = routeWorld([rect(30, 50, 40, 90)], 7, 2, 1);
    const routes = seedRoutes(w, vec(35, 30), vec(35, 110), 3);
    expect(routes.length).toBeGreaterThanOrEqual(2);
    const clear = (route: Vec[]) => {
      const pts = [vec(35, 30), ...route, vec(35, 110)];
      return pts.slice(1).every((p, i) => !w.obstacles.some((o) => segmentHitsConvex(pts[i], p, o)));
    };
    for (const r of routes) expect(clear(r)).toBe(true);
    const sides = routes.map((r) => Math.sign(r[0].x - 35));
    expect(new Set(sides).size).toBe(2);
  });
});
