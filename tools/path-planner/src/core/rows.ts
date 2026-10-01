/** The numbers of a planned path exactly as they are written into Java, and the hash that identifies them. */
import type { PlannedSegment } from './chain.ts';
import { deg, rad } from './geom.ts';

/** Short number for Java source: up to 4 decimals, no trailing zeros. */
export function n(v: number): string {
  if (!Number.isFinite(v)) throw new Error(`can't write ${v} into Java`);
  const s = v.toFixed(4).replace(/\.?0+$/, '');
  return s === '-0' ? '0' : s;
}

/** The numbers of each segment as they appear in the Java, already formatted. */
export function rows(segments: PlannedSegment[]): string[][] {
  return segments.map((seg) => [
    ...seg.curve.flatMap((p) => [n(p.x), n(p.y)]),
    ...seg.heading.F.flatMap((f, i) => [n(f), n(deg(seg.heading.H[i]))]),
  ]);
}

/** Segments exactly as the robot will get them (rounded the way the Java writes them). */
export function roundSegments(segments: PlannedSegment[]): PlannedSegment[] {
  return segments.map((seg) => fromRow(rows([seg])[0].map(Number), seg.leg));
}

export function fromRow(nums: number[], leg: number): PlannedSegment {
  if (nums.length < 12 || nums.length % 2 !== 0) throw new Error('a segment needs 4 control points and at least 2 heading breakpoints');
  const curve = [0, 1, 2, 3].map((i) => ({ x: nums[2 * i], y: nums[2 * i + 1] })) as PlannedSegment['curve'];
  const F: number[] = [];
  const H: number[] = [];
  for (let i = 8; i < nums.length; i += 2) {
    F.push(nums[i]);
    H.push(rad(nums[i + 1]));
  }
  return { curve, heading: { F, H }, leg };
}

/** Identifies the exact geometry; logged with every run so runs can be matched to their path. */
export function geometryHash(segments: PlannedSegment[]): string {
  return fnv1a(rows(segments).map((r) => r.join(',')).join(';')).slice(0, 8);
}


export function fnv1a(text: string): string {
  let h1 = 0x811c9dc5;
  let h2 = 0x01000193;
  for (let i = 0; i < text.length; i++) {
    const c = text.charCodeAt(i);
    h1 = Math.imul(h1 ^ c, 16777619) >>> 0;
    h2 = Math.imul(h2 ^ c, 2246822519) >>> 0;
  }
  return h1.toString(16).padStart(8, '0') + h2.toString(16).padStart(8, '0');
}
