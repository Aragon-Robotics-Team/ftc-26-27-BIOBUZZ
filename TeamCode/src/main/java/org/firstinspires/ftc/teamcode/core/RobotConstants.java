package org.firstinspires.ftc.teamcode.core;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Every robot tunable and hardware name in one place. Drivetrain / localizer config lives in pedro.Constants. */
public class RobotConstants {
    public static class Drive {
        public static double SLOW_MODE_SCALE = 0.4;

        /** How the TeleOp right stick turns the robot. */
        public enum TurnMode {
            TWIN_STICK, // the intake turns to face where the right stick points, and holds that heading when let go
            RATE        // right stick x sets the turn speed
        }
        public static TurnMode TURN_MODE = TurnMode.RATE; // Oct 26: normal turning for now, TWIN_STICK once the heading PID is tuned

        // ---- Twin stick heading hold ----
        // The right stick sets a new heading only when pushed further than this, so its spring-back to center
        // when let go doesn't
        public static double HEADING_STICK_THRESHOLD = 0.5;
        // Radians counter-clockwise from the front of the robot (Pedro +x) to the intake. The intake is on the front.
        public static double INTAKE_HEADING_OFFSET = 0;
        // TODO: tune with the Heading Tuner OpMode
        public static double HEADING_KP = 1.0;        // turn power per radian of error
        public static double HEADING_KI = 0;          // turn power per radian-second
        public static double HEADING_KD = 0;          // turn power per radian/sec of turn speed
        public static double HEADING_MAX_POWER = 1.0; // full power: get to the heading as fast as possible
    }

    public static class Intake {
        public static String MOTOR_NAME = "intake";
        public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE; // Oct 26

        public static double INTAKE_POWER = 1.0;
        public static double OUTTAKE_POWER = -1.0;
        // While shooting. The intake drives the transfer, so this sets how fast balls go into the flywheel (Oct 28)
        public static double FEED_POWER = 0.7;
        public static double FLOWER_INTAKE_MS = 300; // how long Auto waits at a flower with the intake on TODO tune
    }

    public static class Gate {
        public static String SERVO_NAME = "gate";

        // TODO: tune
        public static double CLOSED_POSITION = 0.25;
        public static double OPEN_POSITION = 0.4;
        public static double AUTO_SHOOT_MS = 2000; // how long Auto holds the gate open to empty the robot
    }

    public static class Led {
        public static String SERVO_NAME = "led";
        public static double FLASH_MS = 300;
    }

    /** PsiKit logging of the robot's state for AdvantageScope (util/MatchLog). */
    public static class Logging {
        // Write an .rlog of every TeleOp and Auto run to the Control Hub (FIRST/data/logs). Off until TeleOp testing:
        // it costs loop time and nothing reads the logs yet. The sim logs the robot's state whatever this says.
        public static boolean ENABLED = false;
    }

    /** Hive state from the Limelight 3A on the turret (HiveVision). */
    public static class Vision {
        public static String LIMELIGHT_NAME = "limelight";
        // AprilTag pipeline on the Limelight: family 36h11, tag size 82.55 mm, Full 3D on, LEDs off.
        public static int PIPELINE = 0;
        public static int POLL_RATE_HZ = 100;
        // Mount, landscape on the turret, 4.8 in ahead of its axis (CAD, Oct 26). Calibrate the tilt with the Hive Vision
        // Test OpMode
        public static double LENS_HEIGHT = 12.2;           // inches from the tiles to the lens
        public static double TILT = Math.toRadians(45);    // above horizontal
        // Up-cell tags sit about 49.7 in above the tiles, down-cell tags about 35.5 in. TODO: measure on the practice hive
        public static double HEIGHT_THRESHOLD = 42;        // inches
        public static int FRAMES_TO_SWITCH = 3;            // new frames in a row showing the other state
        public static long MAX_STALENESS_MS = 100;         // ignore results older than this
        // After the operator sets the hive by hand, the camera needs this long of steady disagreement to overrule it
        public static double MANUAL_HOLDOFF_S = 2;
        public static double MANUAL_DISAGREE_S = 1;
        // Operator: hold square / circle this long to set that cell and lock (or unlock) automatic hive detection
        public static double HOLD_TO_LOCK_S = 1;
        public static int LOCK_RUMBLE_MS = 250;
    }

    /** Balls on the tiles from the goBILDA global shutter webcam by the intake (BallVision), and chasing pollen with it. */
    public static class BallVision {
        public static String WEBCAM_NAME = "webcam";
        // The camera's MJPEG modes are 1280x800, 1280x720, 800x600 and 320x240; only 1280x800 is the whole sensor.
        // Frames come in at CAPTURE size and are shrunk to FRAME size (same aspect) before the pipeline.
        public static int CAPTURE_WIDTH = 1280;
        public static int CAPTURE_HEIGHT = 800;
        public static int FRAME_WIDTH = 640;
        public static int FRAME_HEIGHT = 400;
        // Manual exposure keeps motion blur down while driving; gain makes up the brightness.
        // TODO: tune on the robot. 0 leaves the camera on auto exposure
        public static double EXPOSURE_MS = 6;
        public static int GAIN = 100;          // clamped to what the camera allows

        // ---- Lens ----
        // Nominal horizontal field of view, used when CAMERA_MATRIX is null
        public static double HORIZONTAL_FOV = Math.toRadians(70);
        // goBILDA's factory calibration for this camera (3122-0004-0001, USB 0C45:0366, teamwebcamcalibrations.xml,
        // 1280x800 entry), scaled to FRAME_WIDTH x HEIGHT. A camera_calib.json from biobuzz_detection's
        // calibrate_camera.py goes here the same way. null = uncalibrated: HORIZONTAL_FOV, and frames aren't undistorted
        public static int CALIBRATION_WIDTH = 1280;
        public static int CALIBRATION_HEIGHT = 800;
        public static double[] CAMERA_MATRIX = {908.758, 908.758, 696.345, 376.979}; // fx, fy, cx, cy
        public static double[] DISTORTION = {0.0374901, -0.0506747, 0, 0, -0.00229137}; // k1, k2, p1, p2, k3

        // ---- Mount ----
        // On the printed mount (Part 36) at the front of the intake's right side (CAD "02 - Intake", Oct 27). The camera
        // is tilted 30° down, then turned 20° in toward the center line about the vertical, so it looks 20° left of
        // straight ahead, 30° down, and isn't rolled. Lens position from the robot center (between the wheels, on the
        // tiles), inches: the front of the lens in goBILDA's camera CAD, 1 mm behind the case's front face. Rows run
        // along the case's long side (landscape), image up toward the top of the robot.
        public static double LENS_FORWARD = 6.17;
        public static double LENS_LEFT = -4.96;
        public static double LENS_HEIGHT = 8.93;
        public static double YAW = Math.toRadians(20);  // + = turned left
        public static double TILT = Math.toRadians(30); // below horizontal
        public static double ROLL = 0;                  // + = the right side of the image is higher

        // ---- Pipeline ---- (BallPipeline; the rest of its settings are in BallPipeline.Settings)
        // Same values as biobuzz_detection's ball_pipeline.py: change them there first and check with its evaluate.py.
        // Hue windows, OpenCV H (0-179). Deliberately broad; the field-contrast check is what rejects the field.
        public static int[][] POLLEN_HUE = {{12, 45}};
        public static int[][] RED_NECTAR_HUE = {{0, 10}, {165, 179}}; // red wraps around 0
        public static int[][] BLUE_NECTAR_HUE = {{88, 132}};
        // A ball pixel's saturation has to be this many times the frame's median (the field's), clamped
        public static double FIELD_CONTRAST_K = 4.5;
        public static double FIELD_CONTRAST_MIN = 75;
        public static double FIELD_CONTRAST_MAX = 130;
        // Size gate: x the pixel diameter a ball should have where it is (from the mount above)
        public static double SIZE_MARGIN_LO = 0.55;
        public static double SIZE_MARGIN_HI = 1.60;
        // Circle test, on the convex hull
        public static double MIN_CIRCULARITY = 0.65;
        public static double MIN_CIRCLE_FILL = 0.60;
        // Recovering a ball merged with tape: share of directions its edge has to be round in, and how round
        public static double MIN_ROUND_FRAC = 0.75;
        public static double ROUND_TOL = 0.05;
        // Straight-line rejection is among the slowest stages; off still rejects most straight clutter
        public static boolean LINE_FILTER = true;

        // ---- Intake ---- (CAD)
        public static double INTAKE_REACH = 7;      // inches from the robot center forward to the intake's mouth
        public static double INTAKE_WIDTH = 10.1;   // inches between the intake's side plates, centered on the robot

        // ---- Target ----
        public static double LOCK_GATE = 6;         // inches: a pollen this close to the target's last spot is the target
        public static double SMOOTHING = 0.5;       // weight of each new sighting in the target's position
        public static double SWITCH_MARGIN = 0.75;  // another pollen takes over when closer than this x the target's distance
        public static int SWITCH_FRAMES = 5;        // ...for this many frames in a row
        // The camera can't see the last few inches before the mouth (with the CAD mount, 2-5 in). A target last seen
        // within NEAR_MOUTH of the mouth, in front of it and touching the edge of the frame (within EDGE_MARGIN_PX),
        // went into that blind spot rather than getting lost.
        public static double NEAR_MOUTH = 12;
        public static double EDGE_MARGIN_PX = 12;

        // ---- Chase ----
        public static double CHASE_MAX_SPEED = 30;  // in/s. TODO: tune
        public static double OVERSHOOT = 4;         // inches the mouth drives past the ball
        public static double REPLAN_DISTANCE = 2;   // inches the target has to move before the path is redone
        public static double LOST_S = 0.5;          // a target out of view this long (not at the intake) is given up
        public static double CONFIRM_S = 0.4;       // after driving over it, how long it has to stay gone to count
        // With no pollen to chase the robot turns in place to look for more: left, the way the camera already looks
        public static double SEARCH_TURN_RATE = Math.toRadians(90); // rad/s. TODO: tune, slow enough not to blur frames
        public static double SEARCH_TURN = Math.toRadians(360);     // turned this far without seeing pollen, the chase ends
        // A search turns from a spot with room to turn (the robot's half diagonal plus KEEP_OUT_MARGIN clear of the
        // walls, the hive's feet and the flowers), driving there first, for up to SEARCH_MOVE_S
        public static double SEARCH_MOVE_S = 3;
        // Stuck: a target the robot gets no closer to for STUCK_S (moved under STUCK_DISTANCE and turned under
        // STUCK_TURN), as against a wall, is skipped for the rest of the chase, with any pollen within SKIP_RADIUS of it
        public static double STUCK_S = 0.75;
        public static double STUCK_DISTANCE = 1;
        public static double STUCK_TURN = Math.toRadians(5);
        public static double SKIP_RADIUS = 4;
    }

    public static class Launcher {
        public static String FLYWHEEL_MOTOR_NAME = "flywheel";
        public static String TURRET_MOTOR_NAME = "turret";

        public static DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction TURRET_DIRECTION = DcMotorSimple.Direction.FORWARD;

        // ---- Flywheel ----
        // goBILDA 6000 RPM: 28 ticks/rev, about 2800 ticks/sec flat out. The counter-rollers are geared off this motor.
        public static double FLYWHEEL_MAX_VELOCITY = 2100; // ticks/sec (Oct 26)
        // Fixed speed for spinUp() and the Flywheel Tuner
        public static double TARGET_VELOCITY = 1609;  // ticks/sec (Oct 26)
        public static double VELOCITY_TOLERANCE = 50; // ticks/sec, when not aiming with ShotSolver. TODO: tune
        public static double VELOCITY_NUDGE = 25;     // ticks/sec per operator press. TODO: tune
        /** How the flywheel holds its speed. Both run in our own loop (subsystems.FlywheelController), not the hub's. */
        public enum FlywheelControl {
            PIDF,     // battery-compensated feedforward plus PID on the speed error
            BANG_BANG // full power below target, the feedforward at or above it
        }
        public static FlywheelControl FLYWHEEL_CONTROL = FlywheelControl.PIDF;
        // Feedforward: power at 12 V to hold a speed, scaled by 12 / battery volts so it holds the same on a tired battery.
        // The Flywheel Auto Tuner prints KV. A little low is fine; too high and the flywheel settles above target.
        public static double FLYWHEEL_KV = 0.000541; // power per tick/sec (Oct 28)
        public static double FLYWHEEL_KS = 0;        // power to overcome friction
        // PID on the speed error. KP = KV is about the hub PIDF that rapid-fired well on Oct 28 (P = F = 16.84 in the
        // hub's units). The Flywheel Auto Tuner finds KP.
        public static double FLYWHEEL_KP = 0.000541; // power per tick/sec of error
        public static double FLYWHEEL_KI = 0;        // power per tick of accumulated error
        public static double FLYWHEEL_KD = 0;        // power per tick/sec²; leave 0: the speed reading is too noisy to differentiate

        // ---- Turret ----
        // Angles are robot-relative radians: 0 = facing the front of the robot, positive = counter-clockwise.
        public static double TURRET_MOTOR_TICKS_PER_REV = 8192; // REV Through Bore on the motor shaft (1:1), in the turret motor's encoder port (Oct 26)
        public static double TURRET_GEAR_RATIO = 100.0 / 30.0;   // motor revs per turret rev (30T:100T)
        // The Through Bore counts down when the turret turns CCW. setDirection(TURRET_DIRECTION) flips the count too,
        // so changing TURRET_DIRECTION means checking this again.
        public static boolean TURRET_ENCODER_REVERSED = false; // Oct 26
        // TODO: set turret soft limits from CAD
        public static double TURRET_MIN_ANGLE = -Math.PI / 2;
        public static double TURRET_MAX_ANGLE = Math.PI / 2;
        // Turret pivot relative to the robot center (inches, +x forward, +y left): the ring gear's axis (CAD, Oct 26)
        public static double TURRET_OFFSET_X = 1.02;
        public static double TURRET_OFFSET_Y = 0;
        // TODO: tune from turret tuner opmode
        public static double TURRET_KP = 1.9;         // power per radian of error (Oct 26)
        public static double TURRET_KI = 0;           // power per radian-second (Oct 26)
        public static double TURRET_KD = 0.0759;           // power per radian/sec of speed error (Oct 26)
        // Static friction: added toward the target, so small errors still move the turret. TODO: add ?
        public static double TURRET_KS = 0;
        public static double TURRET_KS_DEADBAND = Math.toRadians(0.5); // no kS this close, so it doesn't buzz on target
        public static double TURRET_MAX_POWER = 0.8;
        public static double TURRET_ANGLE_TOLERANCE = Math.toRadians(1); // when not aiming with ShotSolver
        public static double TURRET_NUDGE = Math.toRadians(1); // per operator press

        // ---- Tracking ----
        // Velocity feedforward, so the turret keeps up while the robot drives and turns instead of lagging behind.
        // Power per radian/sec of turret speed. 1 / free speed is a good start: 1150 RPM / (100/30) ≈ 36 rad/s. TODO: tune
        public static double TURRET_KV = 1.0 / 36;
        // Aim from where the robot will be this far ahead, to make up for loop and localizer lag (seconds). TODO: tune
        public static double AIM_LOOKAHEAD = 0.05;

        // ---- Shot model (ShotSolver) ----
        // Every shot is simulated (gravity, drag, and lift if the ball spins) to find the exit speed and turret angle that put the
        // ball through the middle of the opening, allowing for the robot's motion.
        public static double HOOD_ANGLE = Math.toRadians(74); // fixed hood, above horizontal (Oct 28)
        public static double EXIT_HEIGHT = 10.3543;  // inches above the tiles where the ball leaves (CAD, Oct 26)
        public static double EXIT_RADIUS = 0;   // inches from the turret axis to where the ball leaves. TODO: set from CAD
        // Flywheel ticks/sec per inch/sec of ball exit speed. Depends on wheel size and grip, so it has to be measured:
        // stand still, nudge the flywheel until shots go through the middle of the opening, and read "Calibration" in
        // the TeleOp details. Average it over a few distances. Starting value ≈ a 72 mm wheel at 80% grip.
        // TODO: calibrate
        public static double FLYWHEEL_TICKS_PER_EXIT_SPEED = 8.0;
        // The ball keeps the robot's velocity when it leaves; false simulates every shot from a standstill
        public static boolean SHOOT_ON_THE_MOVE = true;
        // Pollen, from the AndyMark product page. Nectar is bigger and heavier in proportion, so it flies the same.
        public static double BALL_DIAMETER = 2.8; // inches
        public static double BALL_MASS = 0.0249;  // kg
        // Air: drag 0.4-0.6 for a smooth sphere. TODO: fit to real shots
        public static double DRAG_COEFFICIENT = 0.5;
        // No lift: the counter-rollers send the ball out with essentially no spin (Oct 28). A launcher that left
        // backspin on it would give lift of about 0-0.45
        public static double LIFT_COEFFICIENT = 0;
        public static double AIR_DENSITY = 1.2;   // kg/m³
        public static double SCORING_MARGIN = 0.5; // inches the ball's edge must clear each side of the opening by
        // Ready to shoot once flywheel and turret errors are within this fraction of what the shot can take
        public static double READY_WINDOW_FRACTION = 0.5;
    }

    /** Red-side poses in Pedro coordinates (inches). Blue is rotated 180° with Alliance.apply; never write blue poses. */
    public static class Field {
        // Center of each cell's opening, from the red driver station's point of view, with the heading pointing out of
        // the opening. The hive is 12.75 in from field center toward our wall, and the up cell's
        // opening is centered 15.7 in out from the hive center. Checked Oct 28
        public static Pose RED_HIVE_LEFT_CELL = new Pose(59.25, 87.7, Math.toRadians(90));   // rear cell
        public static Pose RED_HIVE_RIGHT_CELL = new Pose(59.25, 56.3, Math.toRadians(-90)); // audience cell
        // The up cell's opening (Competition Manual 9.6): 20 in wide, 14 in tall, centered 59.6 in
        // above the tiles, and leaning back 30° with the arm, so its face points outward and 30° up
        public static double HIVE_OPENING_CENTER_Z = 59.6;
        public static double HIVE_OPENING_WIDTH = 20;
        public static double HIVE_OPENING_HEIGHT = 14;
        public static double HIVE_OPENING_TILT = Math.toRadians(30);

        // Flower column centers, one on each perimeter wall 48 in from a corner, about 3.5 in off the wall. The set looks the same from both alliances, so it's used as is for blue.
        // TODO: check against the real field
        public static Pose[] FLOWERS = {
                new Pose(48, 140.5),  // rear wall
                new Pose(96, 3.5),    // audience wall
                new Pose(3.5, 48),    // red alliance wall
                new Pose(140.5, 96),  // blue alliance wall
        };

        // Where the pollen chase won't go for a ball. Both are symmetric about the field center, so they're the same
        // for blue. The hive's frame stands on two foot bars, 2 in wide and 41 in long, 49.5 in apart across the
        // hive; robots drive between them. TODO: check against the real field
        public static double[][] HIVE_FEET = {     // x0, y0, x1, y1
                {46.27, 51.5, 48.27, 92.5},
                {95.73, 51.5, 97.73, 92.5},
        };
        public static double FLOWER_RADIUS = 2.65;   // a flower column's outside. TODO: check against the real field
        public static double FLOWER_KEEP_OUT = 5;  // inches from a flower column's center: inside it, or only the bottom one can be taken
        public static double KEEP_OUT_MARGIN = 2;  // inches added around the hive's feet

        // Robot frame, inches: width side to side, length front to back.
        public static double ROBOT_WIDTH = 15.748; //see below (incldues number sign) Oct 26
        public static double ROBOT_LENGTH = 12.2441; //measured from cad OCT 26- michael

        // Pose resets: robot pushed into the corner, back against the alliance wall, so the center is half the
        // length off the alliance wall and half the width off the side wall.
        public static Pose RED_LEFT_CORNER =  // tile A6, alliance wall x rear wall
                new Pose(ROBOT_LENGTH / 2, 144 - ROBOT_WIDTH / 2, 0);
        public static Pose RED_RIGHT_CORNER = // tile A1, alliance wall x audience wall
                new Pose(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2, 0);
    }
}
