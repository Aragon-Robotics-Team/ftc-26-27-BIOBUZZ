package org.firstinspires.ftc.teamcode.core;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
        public static int FRAME_WIDTH = 640;  // the sensor's 1280x800 binned, so the field of view is the same
        public static int FRAME_HEIGHT = 400;
        // Manual exposure keeps motion blur down while driving; gain makes up the brightness.
        // TODO: tune on the robot. 0 leaves the camera on auto exposure
        public static double EXPOSURE_MS = 6;
        public static int GAIN = 100;          // clamped to what the camera allows

        // ---- Lens ----
        // Nominal horizontal field of view, used until there's a calibration
        public static double HORIZONTAL_FOV = Math.toRadians(70);
        // From biobuzz_detection's calibrate_camera.py (camera_calib.json), solved at CALIBRATION_WIDTH x HEIGHT.
        // TODO: calibrate. null = uncalibrated: HORIZONTAL_FOV, and frames aren't undistorted
        public static int CALIBRATION_WIDTH = 1280;
        public static int CALIBRATION_HEIGHT = 800;
        public static double[] CAMERA_MATRIX = null; // fx, fy, cx, cy
        public static double[] DISTORTION = null;    // k1, k2, p1, p2, k3

        // ---- Mount ----
        // On the printed arducam mount at the front of the intake's left side plate (CAD, Oct 26). The plate is turned
        // 20° in toward the center line, then tipped 23.92° down about the robot's left-right axis, which leaves the
        // camera looking 21.71° right of straight ahead, 22.39° down, and rolled 8.63°. Lens position from the robot
        // center (between the wheels, on the tiles), inches: the middle of the plate's camera face. The lens itself
        // sits a little further out along the view, by the camera's depth.
        // Rows are assumed to run along the plate's long side (landscape).
        public static double LENS_FORWARD = 5.54;
        public static double LENS_LEFT = 5.27;
        public static double LENS_HEIGHT = 7.12;
        public static double YAW = Math.toRadians(-21.71);  // + = turned left
        public static double TILT = Math.toRadians(22.39);  // below horizontal
        public static double ROLL = Math.toRadians(8.63);   // + = the right side of the image is higher

        // ---- TEMPORARY yellow-only detector (BallPipeline) ----
        // Stand-in until ball_pipeline.py is ported: an HSV threshold for POLLEN only, no nectar, no lighting
        // correction. OpenCV HSV: hue 0-180, saturation and value 0-255. TODO: tune on real photos from the C270
        public static double POLLEN_HUE_MIN = 18;
        public static double POLLEN_HUE_MAX = 38;
        public static double POLLEN_SAT_MIN = 110;
        public static double POLLEN_VAL_MIN = 90;
        public static double MIN_FILL = 0.55;        // blob area / its enclosing circle's area; a ball is ~0.8, tape ~0.2
        public static double SIZE_GATE_MIN = 0.5;    // blob diameter / the diameter a pollen there should have
        public static double SIZE_GATE_MAX = 1.6;
        public static double MAX_DEPTH = 96;         // inches along the view; rows that can only hold farther balls are skipped

        // ---- Intake ---- (CAD)
        public static double INTAKE_REACH = 7;     // inches from the robot center forward to the intake's mouth
        public static double INTAKE_WIDTH = 10.1;   // inches between the intake's side plates, centered on the robot

        // ---- Target ----
        public static double LOCK_GATE = 6;         // inches: a pollen this close to the target's last spot is the target
        public static double SMOOTHING = 0.5;       // weight of each new sighting in the target's position
        public static double SWITCH_MARGIN = 0.75;  // another pollen takes over when closer than this x the target's distance
        public static int SWITCH_FRAMES = 5;        // ...for this many frames in a row
        // The camera can't see the last few inches before the mouth (with the CAD mount, 3-4 in). A target last seen
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
        public static double GIVE_UP_S = 2;         // with no pollen in view this long, the chase ends
    }

    public static class Launcher {
        public static String FLYWHEEL_MOTOR_NAME = "flywheel";
        public static String TURRET_MOTOR_NAME = "turret";

        // TODO: set directions (+ power = shoot / turret CCW)
        public static DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction TURRET_DIRECTION = DcMotorSimple.Direction.FORWARD;

        // ---- Flywheel ----
        // goBILDA 6000 RPM: 28 ticks/rev, about 2800 ticks/sec flat out. The counter-rollers are geared off this motor.
        public static double FLYWHEEL_MAX_VELOCITY = 1709; // ticks/sec (Oct 26)
        // Fixed speed for spinUp() and the Flywheel Tuner
        public static double TARGET_VELOCITY = 1609;  // ticks/sec (Oct 26)
        public static double VELOCITY_TOLERANCE = 50; // ticks/sec, when not aiming with ShotSolver. TODO: tune
        public static double VELOCITY_NUDGE = 25;     // ticks/sec per operator press. TODO: tune
        // Velocity PIDF on the hub: F ≈ 32767 / 2800 max ticks/sec, P ≈ F / 10
        public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(1.2, 0, 0, 17.26); // Oct 26. TODO: retune wednesday

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
        // Every shot is simulated (gravity, drag, backspin lift) to find the exit speed and turret angle that put the
        // ball through the middle of the opening, allowing for the robot's motion.
        public static double HOOD_ANGLE = Math.toRadians(67.608); // fixed hood, above horizontal
        public static double EXIT_HEIGHT = 10.3543;  // inches above the tiles where the ball leaves (Oct 26). TODO: set from CAD
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
        // Air: drag 0.4-0.6 for a smooth sphere; lift from the hood's backspin 0-0.45. TODO: fit to real shots
        public static double DRAG_COEFFICIENT = 0.5;
        public static double LIFT_COEFFICIENT = 0.3;
        public static double AIR_DENSITY = 1.2;   // kg/m³
        public static double SCORING_MARGIN = 0.5; // inches the ball's edge must clear each side of the opening by
        // Ready to shoot once flywheel and turret errors are within this fraction of what the shot can take
        public static double READY_WINDOW_FRACTION = 0.5;
    }

    /** Red-side poses in Pedro coordinates (inches). Blue is rotated 180° with Alliance.apply; never write blue poses. */
    public static class Field {
        // Center of each cell's opening, from the red driver station's point of view, with the heading pointing out of
        // the opening. The hive is 12.75 in from field center toward our wall, and the up cell's
        // opening is centered 15.7 in out from the hive center. TODO: double check
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
