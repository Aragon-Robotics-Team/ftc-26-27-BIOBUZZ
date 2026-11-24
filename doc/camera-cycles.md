# Camera Cycles: on the robot

`routines/CameraCycles.java` (Auto menu: `CAMERA_CYCLES`) scores as many tips as 30 s allows from every pollen source
on our half, then parks at the rear end of the loading zone (`FieldPoses.topPark`).

1. Preloads into the up cell (RIGHT, which has 3 nectar in it) from `rightLaunch`. That tips it.
2. Cycles: it picks the source that gives the most pollen for the time (drive there, take it, drive to the up cell's
   launch spot), takes it, and shoots it into whichever cell is up. Sources:
   - the **rear-wall flower** (`flowerIntake`, tuned on the robot) and the **alliance-wall flower**
     (`wallFlowerIntake`, the same spot turned 90° about its flower): back off 10 in, nose in with the intake on;
   - the **garden**: the ball camera's pollen chase from `gardenLook`;
   - **loose pollen** the camera sees within 40 in, such as what spills out of the cell that goes down at a tip.
3. When no cycle can be shot and still leave time to park, it takes one more load to hold for TeleOp if there's time,
   then parks. It drives out at x = 34 and in from the field side, so it doesn't run along the wall into the partner.

Which cell is up: by count (8 pollen's worth tips a cell). If the partner is set to shoot its preloads (the default),
they're counted into the cell that comes up at our first tip. Hive vision corrects the count whenever it reads the
tags. Arriving at a launch spot and seeing none of our tags (after seeing them earlier) means that cell is down: it
goes to the other one.

The pose is the Pinpoint's alone: this auto doesn't fuse the hive's tags into it (`hiveVision.fuseTags(false)`),
whatever `Localization.FUSE_HIVE_TAGS` says. The Limelight is used only to tell which cell is up.

## Before every match

- **Partner:** in init, gamepad1 **triangle** toggles "partner shoots its preloads after our first tip" or "doesn't
  shoot". The partner should park at the **audience end** of the loading zone (the rear end is ours), and stay out of
  the lane between our launch spots and the rear flower (x 44 to 60 on red) while we drive it, about 3 to 10 s.
- **Start:** `poses.start`, back to the audience wall (the init screen shows the numbers). Turret forward at INIT.
- **Limelight** powered early; the init screen says how many hive tags it sees.

## Tune on the robot

| What | Where | Now |
|---|---|---|
| Flower pickup: all 4 out? | `FLOWER_PUSH_S`, `FLOWER_S`, `FieldPoses.flowerIntake` | 1 s push + 0.8 s. The sim gets all 4. It needs the pose within about 2 in of the tuned spot |
| Alliance-wall flower spot | `FieldPoses.wallFlowerIntake` (worked out from the rear one) | untested on the robot |
| Gate open long enough for 4 | `Gate.AUTO_SHOOT_MS` | 2000 ms; at 0.6 feed power, time 4 leaving and cut it to that plus a little |
| Drive time estimates | `DRIVE_SPEED`, `DRIVE_MARGIN_S` | 30 in/s + 0.5 s |
| Garden chase | `GARDEN_S`, `GARDEN_MORE_S`, `FieldPoses.gardenLook` | 5 s cap, done 1 s after a pickup with nothing more. The look heading now follows `BallVision.YAW` |

The driver station's "Doing" line shows the stage, the time, what it holds, the up cell and its count, and tips.

## In the sim

`CameraCyclesSimTest` (realistic drive). With a partner that shoots after our first tip: 2 tips by about 15 s,
then 4 more pollen held for TeleOp, parked: 56 AUTO points. The same in blue, with a slow ball camera, no tags,
scattered shots, and 9 of 10 varied matches (the 10th tips once). `./replay-auto.sh camera_cycles` runs it
(AutoSimTest, with that partner) and opens it in AdvantageScope. With a partner that only parks,
it tips twice alone (preloads, then both flowers into the rear cell) and parks at about 28 s.

## Known limits

- Three tips don't fit. After the second, the third needs two more loads into the cell that came up.
- The flowers need the pose within about 2 in: an off pose misses the bottom pollen.
- Nothing counts the balls inside the robot. A missed flower still counts 4. The launch-spot tag check catches the
  wrong count on the next shot, at the cost of a drive.
