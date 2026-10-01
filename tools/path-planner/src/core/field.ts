/**
 * The BIOBUZZ field, red side, Pedro frame (inches): x from the red alliance wall, y from the audience wall.
 * Measured from the official field CAD (Onshape v1 / STEP v26-27.2, via the field mesh) and checked against the
 * Competition Manual Fig 9-8. The manual allows ±1 in on field placement; the clearance margins cover it.
 * Blue is the same field rotated 180° about (72, 72), which the robot code does at runtime.
 */
import { convexHull, rect, vec, type Polygon, type Vec } from './geom.ts';

/** Inner faces of the perimeter walls (the tiles span 1.4–142.6 in the Pedro frame). */
export const WALL_MIN = 1.4;
export const WALL_MAX = 142.6;
export const CENTERLINE_X = 72;

/** What the turret aims at for the "turret can reach the hive" rule: the red hive's centre. */
export const RED_HIVE_TARGET: Vec = vec(59.25, 72);

/** Red loading zone (tile A5). Robots may not start in it (G304). */
export const RED_LOADING_ZONE: Polygon = rect(1.4, 95.9, 12.9, 118.6);

export interface Obstacle {
  name: string;
  polygon: Polygon;
}

const FLOWERS: Obstacle[] = [
  { name: 'Flower (rear wall)', polygon: rect(45.3, 136.4, 51.9, 143.8) },
  { name: 'Flower (audience wall)', polygon: rect(92.1, 0.2, 98.7, 7.6) },
  { name: 'Flower (red wall)', polygon: rect(0.2, 45.3, 7.6, 51.9) },
  { name: 'Flower (blue wall)', polygon: rect(136.4, 92.1, 143.8, 98.7) },
];

export function flowers(): Obstacle[] {
  return FLOWERS;
}

/** Where a hive frame leg's centreline is at height h, red side. */
function legCenter(h: number, rear: boolean): Vec {
  return vec(48.06 + 0.2729 * h, rear ? 90.8 - 0.4452 * h : 53.2 + 0.4452 * h);
}

/** A horizontal slice through a 1 in square leg tube, which leans, is about 1.2 (x) by 1.16 (y). */
function legSlice(c: Vec): Vec[] {
  return [vec(c.x - 0.6, c.y - 0.58), vec(c.x + 0.6, c.y - 0.58), vec(c.x + 0.6, c.y + 0.58), vec(c.x - 0.6, c.y + 0.58)];
}

const LEG_BASE_HEIGHT = 2;
const LEG_APEX_HEIGHT = 42;
const HIVE_BOTTOM = 30.65;
const LOGO_PANEL_BOTTOM = 34.1;

const mirrorX = (poly: Polygon): Polygon => convexHull(poly.map((p) => vec(144 - p.x, p.y)));

const obstacleCache = new Map<number, Obstacle[]>();

/**
 * Every fixed obstacle a robot whose top (plus headroom) reaches `top` inches can hit.
 * Hive frame: the foot bar is solid at any height; each leg blocks the floor area under the part of it that is lower
 * than `top`, which is the sweep of its slice from the foot (h = 2) up to `top`.
 */
export function fieldObstacles(top: number): Obstacle[] {
  const cached = obstacleCache.get(top);
  if (cached) return cached;
  const obstacles: Obstacle[] = [...FLOWERS];
  const legTop = Math.min(Math.max(top, LEG_BASE_HEIGHT), LEG_APEX_HEIGHT);
  const redFrame: Obstacle[] = [
    { name: 'Hive frame foot bar', polygon: rect(47.25, 52.5, 49.5, 91.5) },
    {
      name: 'Hive frame leg (audience)',
      polygon: convexHull([...legSlice(legCenter(LEG_BASE_HEIGHT, false)), ...legSlice(legCenter(legTop, false))]),
    },
    {
      name: 'Hive frame leg (rear)',
      polygon: convexHull([...legSlice(legCenter(LEG_BASE_HEIGHT, true)), ...legSlice(legCenter(legTop, true))]),
    },
  ];
  obstacles.push(...redFrame);
  // The blue side of the frame is the red side mirrored across the centerline.
  for (const o of redFrame) obstacles.push({ name: `${o.name}, blue side`, polygon: mirrorX(o.polygon) });
  if (top >= HIVE_BOTTOM) {
    obstacles.push({ name: 'Red hive', polygon: rect(48.8, 53, 69.8, 91) });
    obstacles.push({ name: 'Blue hive', polygon: rect(74.3, 53, 95.3, 91) });
  }
  if (top >= LOGO_PANEL_BOTTOM) obstacles.push({ name: 'Hive logo panels', polygon: rect(56.9, 67.7, 87.1, 76.3) });
  obstacleCache.set(top, obstacles);
  return obstacles;
}

/** Tape-only areas, drawn for reference. */
export const FIELD_MARKINGS: { name: string; polygon: Polygon }[] = [
  { name: 'Red loading zone', polygon: RED_LOADING_ZONE },
  { name: 'Blue loading zone', polygon: rect(131.1, 25.4, 142.6, 48.1) },
  { name: 'Red garden', polygon: rect(1.9, 1.9, 24.6, 3.9) },
  { name: 'Blue garden', polygon: rect(119.4, 140.1, 142.1, 142.1) },
];
