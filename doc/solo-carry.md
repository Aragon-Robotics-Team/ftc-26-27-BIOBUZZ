# Solo Carry: on the robot

`routines/SoloCarry.java` (Auto menu: `SOLO_CARRY`) scores alone with a partner that only parks. It shoots the 4
preloads into the up cell (with the 3 staged nectar, that tips it), chases pollen with the ball camera (the garden first),
shoots it into the cell that's up now, and parks. The sim runs it in 19 scenarios (`SoloCarrySimTest`): 39 to 47
points, both robots parked in all (the flowers' pollen stays in the flowers; the intake only takes a ball whose
center is between its side plates). None of its timing or noise numbers have been checked on the
robot yet. This is what to check, roughly in order.

## Before every match

- **Start pose:** back against the alliance wall on tile A2, facing the field (`FieldPoses.allianceWallStart`; the init
  screen shows the numbers). Set down within an inch or two: the sim's off-by-1.5 in, 5° runs still scored 49.
- **Turret zeroed** (the init screen asks). A turret zero error goes straight into the fused heading.
- **Limelight:** the init screen says how many hive tags it sees. If it says none, turn the turret toward the hive by
  hand: still none means it's unplugged, booting, or on the wrong pipeline (it retries the switch to
  `Vision.PIPELINE` once a second). Without tags Solo Carry still runs on its count, but a drifting IMU then goes
  uncorrected (the sim's worst case: 16 in off by the end).
- **Partner:** starts in our LEFT corner (back to the alliance wall by the loading zone) and parks **flush at the rear
  end of the loading zone**. We park at its audience end, nose 2 in into the zone, which leaves about 3 in between
  us and an 18 in partner parked flush at the rear. Anywhere else in the zone and we hit it.

## Tune on the robot

| What | Where | How to measure | Now |
|---|---|---|---|
| Drive speed for the time budget | `SoloCarry.DRIVE_SPEED`, `DRIVE_MARGIN_S` | Time `driveTo` from rightLaunch to leftLaunch, and to the park spot | 30 in/s + 0.5 s (the sim does about 40) |
| Launcher ready time at the spot | `SoloCarry.READY_TYPICAL_S` (planning), `READY_WAIT_S` (cap) | `SoloCarry/stage` "to the launch spot" to "shooting" in the log | 0.5 s planned, 1.5 s cap |
| Shot speed calibration | `Launcher.FLYWHEEL_TICKS_PER_EXIT_SPEED`, the ShotSolver's drag | Shoot from both launch spots, see where they land against the opening | The sim's shots are right by construction (the solver uses its drag). Every shot 5% off still tips the hive; 8% off, the preload misses it (24 to 26 points) |
| Gate open long enough to empty 4 | `Gate.AUTO_SHOOT_MS` | Shoot 4, watch the last one leave | 2000 ms (the sim feeds one per 0.26 s) |
| Up or down from tag height | `Vision.HEIGHT_THRESHOLD` | The `Hive tags` telemetry line (HiveVision, in TeleOp) with each cell up, both hives | 42 in (TODO in the code) |
| Hive tag positions | `core/HiveTags` (pivot height, arm length, tag offsets) | Tape measure on a real hive, at rest both ways | from the manual's drawings |
| Odometry drift | `Localization.TRANSLATION_DRIFT`, `HEADING_DRIFT_*` | Drive a known square with tags off (`FUSE_HIVE_TAGS = false`), log the Pinpoint's error | placeholders |
| Tag noise | `Localization.TAG_ANGLE_SIGMA`, `TAG_RANGE_FRACTION` | Robot still, log `getTargetPoseCameraSpace` for 10 s at 3 distances | placeholders |
| Limelight latency | the fake's 20 + 20 ms in `BiobuzzRobotSim` | `getCaptureLatency()`, `getTargetingLatency()` on the robot | sim only |
| Ball pickup counting | `PollenChase` (`CONFIRM_S`, the mouth window, `FULL_REFUSALS`) | Run the Ball Chase Test over a row of 3; compare `Chase/status` "picked up" with what's inside | sim-tuned |
| Hunt for one more | `SoloCarry.MORE_S` | How long the 4th ball takes after the 3rd | 5 s (sim: no gain past 5) |

After a practice run, open the log in AdvantageScope and compare `Drive/pose` (fused) against `Drive/odometryPose`
(the Pinpoint alone), especially while turning fast in front of the hive. A fused heading that jumps there means the
tags are being applied at the wrong moment: the SDK re-reads a frame every 10 ms and stamps it from the latest read,
so frames can be up to about 20 ms older than they say.

## How it decides, and its margins

- **Which cell is up:** at a cell's launch spot the tags face it only while that cell is up. After a shot the count says
  should tip it, tags gone for 0.6 s means tipped. Tags missing on arrival only send it to the other cell when the
  count is within 2 of a tip, once a load (so a Limelight that stops seeing can't send it back and forth). With no
  tags at the spot, it goes by the count, which errs high.
- **Balls held:** counted from the camera, erring high (every pollen the mouth passes over, and two that won't go in
  mean full). Shooting holds the gate open long enough to empty the robot whatever the count.
- **Time:** it counts 29 s. Each chase leaves time to drive to the launch spot, shoot, and park from there, plus
  1.5 s. Parking starts when the drive there (by an approach point on the audience side, clear of the partner) is all
  that's left, plus 1 s. Every drive gives up after twice its estimate and holds where it is.

## Known limits

- One tip in 30 s is all there is to get: a second needs 8 more pollen in the new up cell, and our side only has the
  4 garden balls and whatever of our 4 preloads spills our way (some crosses the center line). In the sim it gets one
  load of 3 or 4 in after the tip; the chase from the other launch spot then finds nothing to take.
- The scoring, Park and Leave rules in the sim come from BiobuzzSim (the Roblox sim), not the manual: check them,
  and whether garden balls count at the end of AUTO (then taking them is +1 each) or only at the end of the match.
- Also check whether robots may cross the center line in AUTO: the chase goes after any pollen on open floor
  (`BallVision.onOpenFloor`), our half or not. If there's a rule, add the half there.
- With no tags at all and the IMU drifting, nothing corrects the heading; the park can miss the zone.
- It can't take pollen from the flowers.
