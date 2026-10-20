import { defaultSettings, type Chain, type Settings } from '../../src/core/project.ts';

/** The CycleGardenPark moves, with poses that fit a 16 × 14 robot inside the CAD walls. */
export function garden(): { settings: Settings; chains: Chain[] } {
  const launch = { kind: 'fixed' as const, x: 56, y: 20.22, heading: 90, headingTol: 0 };
  return {
    settings: defaultSettings(),
    chains: [
      {
        name: 'gardenCycle',
        points: [
          launch,
          { kind: 'fixed', x: 28.4, y: 12, heading: 180, headingTol: 10 },
          { kind: 'fixed', x: 12, y: 12, heading: 180, headingTol: 10 },
        ],
        // Into the garden corner, intake first. It backs out again on a path of its own: Pedro only stops at the end.
        legs: [{ rules: [] }, { rules: [{ type: 'front-first', tol: 15 }] }],
        endStopped: true,
        matchStart: false,
        markers: [{ name: 'INTAKE_ON', leg: 0, at: 0.6 }],
        keepOut: [],
      },
      {
        name: 'park',
        points: [launch, { kind: 'fixed', x: 14, y: 100, heading: 90, headingTol: 0 }],
        legs: [{ rules: [] }],
        endStopped: true,
        matchStart: false,
        markers: [],
        keepOut: [],
      },
    ],
  };
}

/** A match start pose touching the audience wall, as its own one-leg path. */
export function startChain(x: number, y: number, heading: number): Chain {
  return {
    name: 'fromStart',
    points: [
      { kind: 'fixed', x, y, heading, headingTol: 0 },
      { kind: 'fixed', x: 56, y: 30, heading: 90, headingTol: 0 },
    ],
    legs: [{ rules: [] }],
    endStopped: true,
    matchStart: true,
    markers: [],
    keepOut: [],
  };
}
