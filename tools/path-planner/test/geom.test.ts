import { describe, expect, it } from 'vitest';
import { convexDistance, convexHull, offsetConvex, rect, segmentHitsConvex, vec, wrap } from '../src/core/geom.ts';

describe('geometry', () => {
  it('measures the gap between separated boxes, including corner to corner', () => {
    expect(convexDistance(rect(0, 0, 1, 1), rect(3, 0, 4, 1))).toBeCloseTo(2);
    expect(convexDistance(rect(0, 0, 1, 1), rect(4, 5, 5, 6))).toBeCloseTo(5); // 3-4-5 corners
  });

  it('reports overlap as a negative gap', () => {
    expect(convexDistance(rect(0, 0, 2, 2), rect(1.5, 0, 3, 2))).toBeCloseTo(-0.5);
  });

  it('tells which segments cross a polygon', () => {
    const box = rect(0, 0, 2, 2);
    expect(segmentHitsConvex(vec(-1, 1), vec(3, 1), box)).toBe(true);
    expect(segmentHitsConvex(vec(-1, 3), vec(3, 3), box)).toBe(false);
    expect(segmentHitsConvex(vec(-1, 2), vec(3, 2), box)).toBe(false); // grazing an edge is allowed
  });

  it('grows a polygon by r on every side', () => {
    const grown = offsetConvex(rect(0, 0, 2, 2), 1);
    expect(convexHull(grown)).toEqual(convexHull(rect(-1, -1, 3, 3)));
  });

  it('wraps angles into (-π, π]', () => {
    expect(wrap(3 * Math.PI)).toBeCloseTo(Math.PI);
    expect(wrap(-Math.PI / 2 - 2 * Math.PI)).toBeCloseTo(-Math.PI / 2);
  });
});
