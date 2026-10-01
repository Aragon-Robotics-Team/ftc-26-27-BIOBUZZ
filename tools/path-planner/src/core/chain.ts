/** A concrete planned chain (Bezier segments + heading pieces) and its samples along arc length. */
import { curvature, derivative, point, tableFor, type ArcTable, type Cubic } from './bezier.ts';
import { angleOf, type Vec } from './geom.ts';
import { pedroHeading, type HeadingPieces } from './heading.ts';

export interface PlannedSegment {
  curve: Cubic;
  heading: HeadingPieces;
  /** Which leg of the chain spec this segment belongs to. */
  leg: number;
}

export interface Sample {
  seg: number;
  leg: number;
  /** Curve parameter within the segment. */
  t: number;
  /** Arc length from the start of the chain. */
  s: number;
  p: Vec;
  /** Direction of travel. */
  travel: number;
  /** Signed curvature, 1/in. */
  kappa: number;
  /** Heading, as Pedro will command it (unwrapped). */
  h: number;
}

export interface SampledChain {
  samples: Sample[];
  tables: ArcTable[];
  /** Arc length at the start of each segment, plus the total at the end. */
  segStart: number[];
  length: number;
}

/** Samples every `ds` inches of arc length (and at every segment end). */
export function sampleChain(segments: PlannedSegment[], ds = 0.5): SampledChain {
  const samples: Sample[] = [];
  const tables: ArcTable[] = segments.map((s) => tableFor(s.curve));
  const segStart: number[] = [];
  let s0 = 0;
  segments.forEach((seg, i) => {
    const table = tables[i];
    segStart.push(s0);
    const n = Math.max(2, Math.ceil(table.length / ds));
    for (let j = i === 0 ? 0 : 1; j <= n; j++) {
      const c = j / n;
      const t = table.parameter(c);
      const d = derivative(seg.curve, t);
      samples.push({
        seg: i,
        leg: seg.leg,
        t,
        s: s0 + c * table.length,
        p: point(seg.curve, t),
        travel: d.x * d.x + d.y * d.y > 1e-18 ? angleOf(d) : samples.length ? samples[samples.length - 1].travel : 0,
        kappa: curvature(seg.curve, t),
        h: pedroHeading(table, seg.heading, t),
      });
    }
    s0 += table.length;
  });
  segStart.push(s0);
  // Keep headings continuous across segments (each segment's pieces are unwrapped on their own).
  for (let i = 1; i < samples.length; i++) {
    const prev = samples[i - 1].h;
    let h = samples[i].h;
    while (h - prev > Math.PI) h -= 2 * Math.PI;
    while (h - prev < -Math.PI) h += 2 * Math.PI;
    samples[i].h = h;
  }
  return { samples, tables, segStart, length: s0 };
}
