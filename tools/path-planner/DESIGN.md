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

The planner predicts how Pedro (Foresight, 3.0.1) would drive a path; it doesn't replace Pedro. Two parts:

**What the drivetrain can do** (`src/core/drivetrain.ts`), from the motor (free speed, stall torque), gearing, wheel
size, wheelbase and track width, mass and grip. Each wheel's motor has a voltage budget shared between spinning and
pushing, and mecanum wheels add up what forward, sideways and turning ask of them, per axis and with sign:

- `|vx/vF + ax/aF| + |vy/vS + ay/aS| + |ω/ωMax + α/αMax| ≤ battery / spec voltage` (x = robot forward, y = left)
- `|ax| + |ay| + |α|·I/(m·R) ≤ grip · g`

So acceleration falls with speed (motor torque does), strafing is slower by the strafe efficiency, and turning takes
both top speed and acceleration. Top speeds come out as free speed × efficiency.

**How Pedro drives** (`src/core/model.ts`, forward and backward pass over 0.5 in samples): corrections first
(centripetal push for the curve, turning to the planned heading), drive power gets what's left, and it brakes only for
the end of the path (never ahead of a curve). Where the corrections don't fit even with drive at zero, the robot
coasts and the push it's missing becomes **drift** wide of the path (and the turning it's missing becomes **heading
lag**); both recover with a 0.3 s time constant. More than 1 in of drift or 5° of lag breaks the plan.

- turning back on itself (more than 90° between two samples) breaks the plan: Pedro only brakes at the end of a path,
  so it would overshoot. To stop somewhere, end the path there and start another
- speeds scale with battery voltage (12.5 V assumed) / the motors' spec voltage
- a fixed overhead per stop (settling at the end of a chain)

Numbers start as a sample: goBILDA 312 RPM motors on 104 mm mecanum wheels, with mass, top speeds and coasting from
the Pedro quickstart's tuned `FollowerConstants`; grip 0.5 and 90 % drive efficiency are typical guesses. Pasting the
Foresight Tuner output sets the efficiencies and coasting from measured values; fitting logs adjusts efficiencies,
grip, mass, coasting and settling. When imported logs disagree with the model by more than 5 % across several runs,
the planner flags it; refitting is a manual step. If logs show Pedro doing something this doesn't capture, the next
step is to simulate Pedro's own control loop.

## Shooting

A point (or area) can shoot at a hive cell, in one of two ways:

- **Stop and shoot**: the robot stops there, waits until the launcher is ready, fires a volley, and drives on. Pedro
  only stops at the end of a path, so the path is driven as separate Pedro paths with `robot.shoot()` between them;
  the robot may leave a stop in any direction. Allowed on any point, including the start and the end.
- **Shoot on the move**: the volley is centred on a point in the middle of the path, on a stretch where Pedro's speed
  is capped (`maxPathSpeed`, set per segment). Pedro doesn't slow down ahead of a cap, and on a capped segment it
  coasts while too fast, so the capped stretch starts where coasting from the planned speed reaches the cap in time.

Shots are simulated with TeamCode's `ShotSolver`, ported line for line (`src/core/shot.ts`): ball flight with drag and
backspin, launched with the robot's velocity, giving exit speed, turret angle and how much error each can take. A ball
scores when the robot's **aim accuracy** (± degrees) fits in the angle tolerance and its **flywheel speed accuracy**
(± percent) plus how far the flywheel lags fits in the speed tolerance, and the turret can turn that far.

The flywheel (motor torque and the **flywheel inertia**) is followed along the whole path, chasing the speed the next
shot needs; every ball takes energy from it (about 2.5× the ball's kinetic energy), and it has to recover before the
next. Balls leave evenly over the **volley length** (`Gate.AUTO_SHOOT_MS`). A stop waits for the launcher to be ready
(within half the tolerance, as `Launcher.isReady()`); a shot on the move must be ready when the gate opens.

The shaded zone on the field (shown while a shooting point is selected) is where a standing shot scores despite the
robot's errors. Before optimizing, a shooting point that can't score from anywhere it allows, or whose headings keep
the cell out of the turret's range, is flagged.

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
- CMA-ES with a fixed random seed. Every route gets a short screening run (2,500 evaluations); the two most
  promising get two full runs each (the first carrying on from screening), plus up to two more with a bigger
  population if they still break a limit. A route is skipped when even driving its corners flat out can't beat the
  best found. Hard limits become steep penalties and the result must have none left. Paths are scored at 0.5 in
  during the search and in the final report alike; during the search the footprint is checked at every other sample
  with 0.2 in of extra margin. Runs end when the best time stops improving, after 12,000 evaluations, or on Stop.
- Long paths have several good local optima. **Optimize again** on an unchanged path searches with a new seed and
  keeps whichever plan is faster.
- While it runs, the panel shows the route and run, evaluations and elapsed time, the current run's best time and
  whether it fits, and the best fitting time so far.
- Reports: time, closest approach to each obstacle, and which limit is holding it back.

## Robot code

- A path that stops to shoot exports one method per Pedro path (`name`, `name2`, …) and a comment saying how to drive
  them with the volleys between. A shot on the move exports a marker where the gate should open and a comment with
  the `deadline(follow(…), sequential(passed(…), robot.shoot()))` to use. Capped segments start with
  `PlannedPath.SLOW, <in/s>`.
- **.pp** downloads the plan for the Pedro Visualizer: one line per curve, each with a piecewise heading through the
  planned breakpoints (within Pedro's own small warp between them), plus the robot's size and speeds. The visualizer
  times every line as its own start-and-stop move, so its clock and animation speed don't match the plan; the
  planner's playback does.
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
- Runs are matched to their path by the id in `PlannedPath.of` ("name#geometry", or "name.2#geometry" for the second
  Pedro path of one that stops to shoot); if the path has changed since, the planner rebuilds the driven path from
  the logged poses instead.
- The `Planned Path Test` OpMode builds every routine to find the planned paths they use, then drives one and logs it.
