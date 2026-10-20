/** The robot's footprint: the frame rectangle plus any protrusions, each a convex polygon. */
import { add, rect, rotate, type Polygon, type Pose } from './geom.ts';
import type { RobotConfig } from './project.ts';

/** Footprint in the robot frame: +x is the front (intake), +y is the robot's left. */
export function localFootprint(robot: RobotConfig): Polygon[] {
  const hl = robot.length / 2;
  const hw = robot.width / 2;
  const parts: Polygon[] = [rect(-hl, -hw, hl, hw)];
  for (const p of robot.protrusions) {
    switch (p.side) {
      case 'front':
        parts.push(rect(hl, p.from, hl + p.depth, p.to));
        break;
      case 'back':
        parts.push(rect(-hl - p.depth, p.from, -hl, p.to));
        break;
      case 'left':
        parts.push(rect(p.from, hw, p.to, hw + p.depth));
        break;
      case 'right':
        parts.push(rect(p.from, -hw - p.depth, p.to, -hw));
        break;
    }
  }
  return parts;
}

/** The footprint placed at a field pose. */
export function placeFootprint(local: Polygon[], pose: Pose): Polygon[] {
  return local.map((poly) => poly.map((v) => add(rotate(v, pose.h), pose)));
}

/** Distance from the robot centre to its farthest point. */
export function circumradius(local: Polygon[]): number {
  let r = 0;
  for (const poly of local) for (const v of poly) r = Math.max(r, Math.hypot(v.x, v.y));
  return r;
}
