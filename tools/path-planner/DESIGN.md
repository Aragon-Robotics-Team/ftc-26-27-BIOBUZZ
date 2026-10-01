# Path planner: design

The decisions behind the path planner, agreed with the team on 2026-09-30 and revised the same day to copy-paste
export. Change them here first, then in the code.

## Purpose

Describe each auto move as a **spec** (where it starts and ends, what it must pass, which way the robot faces, what to avoid).
The planner finds the fastest path that keeps every clearance margin, writes Java the robot runs, and checks its own
speed model against logs from real runs.

- Authoring comes first: no more hand-placed Bezier curves. Optimized times are labelled experimental until the model
  has been fitted to and checked against logs.
- "Optimal" means: **minimum predicted time for paths of this form (cubic Beziers + heading knots), under our measured
  robot model, with every margin and rule as a hard limit, searched from every way around the obstacles.**

## Stack and distribution

- TypeScript + Vite. The GUI is plain TypeScript and a canvas, styled like the other tools (FTC Dashboard look):
  a big field, one side panel for the selected path, dialogs for code, robot settings and logs.
- `src/core/` has no DOM code; the GUI runs it in a Web Worker (a classic one: Chrome won't start module workers from a
  `file://` page).
- `npm run build` writes one self-contained `tools/path-planner.html`, committed so anyone can double-click it.
- The planner never touches the repo. Work stays in the browser between visits; **the copied code is the record**.
- Tests: vitest in `tools/path-planner/test/`. No Java unit tests.

## Field model (red side; blue is the red plan rotated 180° about (72, 72) at runtime)

Hand-written in `src/core/field.ts`, measured from the official BIOBUZZ field CAD (Onshape v1, STEP v26-27.2) and the
manual (Fig 9-8). Pedro frame, inches: x from the red wall, y from the audience wall. The manual allows ±1 in.

- Walls: inner faces at 1.4 and 142.6.
- Hive A-frames: foot bar (solid, 2.15 in lip) x 47.25–49.5, y 52.5–91.5; legs are 1 in square tube leaning inward,
  centerline x = 48.06 + 0.2729·h, y = 53.20 + 0.4452·h (audience leg) / 90.80 − 0.4452·h (rear leg), from h ≈ 2.
  Keep-out = foot bar + the leg's sweep from h = 2 up to robot height + headroom. Blue's frame is x → 144 − x.
- Hives (bottom 30.65 in), logo panels (34.1 in): obstacles only if the robot top + headroom reaches them.
- Flowers: four solid boxes at the wall centres.
- Centerline: no part of the robot may pass x = 72 (G402). A path can tick "Allow crossing the centerline" (Options) when it isn't for AUTO;
  the far half's obstacles (the blue side of the hive frame, the flowers) still apply.
- Not obstacles: loading zones and gardens (tape). The "goal" shapes in old `.pp` files are from last season.

## Robot

- Footprint: rectangle (16 in wide, 14 in long, 14 in tall), swept at its actual heading. Protrusions are optional
  rectangles on any side. All editable under Robot & speed.
- Turret range: from −90° to 90° of the front by default (counter-clockwise positive), set under Robot & speed.
  "Turret aims at hive" keeps the hive inside it, less the rule's margin; 360° or more means always.
- Margins: 2 in from obstacles and walls, 1 in from the centerline, 2 in of headroom under the hive frame, set for all
  paths under Robot & speed. A path can tick "Use different gaps for this path" (Options) for a tight spot.
- A path can mark its first point as the match start position; it must satisfy G304 (fully on our side, touching a
  wall, not touching a flower, not in the loading zone).

## Speed model

Fastest speed along the path, forward and backward pass:

- direction limit: `|v·cos α|/vForward + |v·sin α|/vStrafe + |ω|/omegaMax ≤ 1` (α = travel direction relative to heading)
- curvature: `v²·|κ| ≤ aLateral`, with κ the sharper of the curvature at a sample and the turn between neighbouring
  samples (so a hook tighter than the sampling can't be taken at speed)
- turning back on itself (more than 90° between two samples) means stopping, halfway between them; the panel then
  says to split the path there, because Pedro only brakes at the end of a path
- acceleration / deceleration, direction-dependent the same way (forward and strafe values)
- speeds scale with battery voltage / nominal voltage
- a fixed overhead per stop (settling at the end of a chain)

Numbers come from pasting the Foresight Tuner output, then from fitting logs. When imported logs disagree with the
model by more than 5 % across several runs, the planner flags it; refitting is a manual step.

## Paths

- A **path** (internally a chain) is a sequence of points optimized together and driven as one Pedro compound path
  (one `follow()`). Each point owns its pose. The first and last are exact points; the ones between are points or
  areas (the robot passes somewhere inside). A path ends stopped or still moving.
- Dragging a point onto another path's start or end snaps to it, so paths join up.
- How the robot faces on each leg: any way, intake first (± tolerance), a fixed heading (± tolerance), or turret can
  aim at the hive (turret is ±90° from the front).
- Markers: named points along a path for triggering mechanism actions.
- Avoid zones per path (e.g. where a partner parks).
- No hand edits of generated code: every "I know better" becomes a point, area, rule or zone.
- Before optimizing, every point is checked on its own: if no position (in an area) and no allowed heading keeps every
  gap, or its heading can't meet a neighbouring leg's facing rule, the point turns red and the panel says why.
  Optimize then asks before going ahead, since the optimizer can't fix it.
- A match start touches the wall (G304), so on such a path the wall margin grows from the start's own gap as the robot
  drives away, reaching the full margin after that many inches.
- Nothing can be edited while a path is optimizing (points, panel, settings, new or pasted paths); Stop ends the run.

## Optimizer

- Seeds: shortest routes through a visibility graph of the inflated obstacle corners, one per distinct way around.
- Variables: cubic Bezier legs with smooth (G1) joins (joint positions, tangent directions, handle lengths) and
  heading knots.
- CMA-ES with a fixed random seed. Each route gets two independent runs, plus up to two more with a bigger population
  if it still breaks a limit; a route is skipped when even driving its corners flat out can't beat the best found.
  Hard limits become steep penalties and the result must have none left. Paths are scored at 0.5 in during the
  search and in the final report alike (scoring coarser made the search favour shortcuts that only looked good at
  the coarse resolution). Runs end when the best time stops improving by 0.1 ms, after 12,000 evaluations, or on Stop.
- While it runs, the panel shows the route and run, evaluations and elapsed time, the current run's best time and
  whether it fits, and the best fitting time so far.
- Reports: time, closest approach to each obstacle, and which limit is holding it back.

## Robot code

- **Copy code** gives a short method to paste into a routine:

  ```java
  // path-planner {"v":1,"chain":{…},"settings":{…},"legs":[…]}
  // gardenCycle: 3.12 s planned. To change it, paste this into tools/path-planner.html.
  public static final PlannedPath.Marker GARDEN_CYCLE_INTAKE_ON = new PlannedPath.Marker(0, 0.6758);
  static Path gardenCycle(PoseFactory f) {
      return PlannedPath.of("gardenCycle#2f2c7217", f, true, new double[] {…}, …);
  }
  ```

  The first line carries the path and the settings it was planned with, so **Paste code** restores it (from one
  method or a whole routine file). Each array is one cubic Bezier (control points, red side) followed by heading
  breakpoints (fraction of the segment, degrees, red side).
- `routines/PlannedPath.java` (hand-written, once) builds the Pedro compound path, flipping for blue through the
  routine's `PoseFactory`. Headings are many short `Interpolator.linear` pieces, because Pedro 3.0.1's `piecewise()`
  evaluates `tangent`/`facingPoint` at the wrong place in every piece but the first (and warps `linear` slightly
  between breakpoints, which the planner reproduces exactly).
- `AutoRoutine.passed(path, marker)` waits until the follower has passed a marker.
- Nothing is generated into TeamCode and there is no build-time check: the pasted code is the source of truth.

## Logging and checking

- Every Auto writes a per-loop CSV through Pedro's `Follower.withLogger` to `/sdcard/FIRST/data/pathlogs/`, one file
  per run, kept forever. Columns include battery voltage.
- Getting logs off the robot: a "Path logs" page on the Robot Controller web server
  (`http://192.168.43.1:8080/pathlogs`), with copying over USB as the fallback.
- The GUI imports logs, plots predicted vs measured, fits the model, and flags drift. Fitting is tested on synthetic
  logs from the model itself.
- Runs are matched to their path by the id in `PlannedPath.of` ("name#geometry"); if the path has changed since, the
  planner rebuilds the driven path from the logged poses instead.
- The `Planned Path Test` OpMode builds every routine to find the planned paths they use, then drives one and logs it.
