package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/** Every robot tunable and hardware name in one place. Drivetrain / localizer config lives in pedro.Constants. */
public class RobotConstants {
    /** Red-side poses in Pedro coordinates (inches). Blue is rotated 180° with Alliance.apply; never write blue poses. */
    public static class Field {
        // Center of each cell's opening, from the red driver station's point of view. From BiobuzzSim: the hive is
        // 12.75 in from field center toward our wall, and the up cell's opening is centered 15.7 in out from the hive
        // center (59.6 in above the tiles, leaning back 30°). TODO: check against the real field
        public static Pose RED_HIVE_LEFT_CELL = new Pose(59.25, 87.7, 0);  // rear cell
        public static Pose RED_HIVE_RIGHT_CELL = new Pose(59.25, 56.3, 0); // audience cell

        // Pose resets: robot pushed into the corner, back against the alliance wall.
        // 9 in = half of an 18 in robot. TODO: use the real robot size from CAD
        public static Pose RED_LEFT_CORNER = new Pose(9, 135, 0); // tile A6, alliance wall x rear wall
        // TODO: check against the real field (the GARDEN is in this corner; clear any balls first)
        public static Pose RED_RIGHT_CORNER = new Pose(9, 9, 0);  // tile A1, alliance wall x audience wall
    }

    public static class Drive {
        public static double SLOW_MODE_SCALE = 0.4; // TODO: tune
    }

    public static class Intake {
        public static String MOTOR_NAME = "intake";
        // TODO: set direction (+ power = intake)
        public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

        // TODO: tune
        public static double INTAKE_POWER = 1.0;
        public static double OUTTAKE_POWER = -1.0;
    }

    public static class Gate {
        public static String SERVO_NAME = "gate";

        // TODO: tune
        public static double CLOSED_POSITION = 0.0;
        public static double OPEN_POSITION = 0.5;
        public static double AUTO_SHOOT_MS = 1000; // how long Auto holds the gate open to empty the robot
    }

    public static class Led {
        public static String SERVO_NAME = "led";
        public static double FLASH_MS = 300;
    }

    public static class Launcher {
        public static String FLYWHEEL_MOTOR_NAME = "flywheel";
        public static String TURRET_MOTOR_NAME = "turret";

        // TODO: set directions (+ power = shoot / turret CCW)
        public static DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction TURRET_DIRECTION = DcMotorSimple.Direction.FORWARD;

        // ---- Flywheel ----
        // goBILDA 6000 RPM: 28 ticks/rev, about 2800 ticks/sec flat out. The counter-rollers are geared off this motor.
        // Speed for a distance to the hive cell: rows of {inches, ticks/sec}, sorted by distance.
        // Linearly interpolated between rows, and held at the end rows outside them.
        // TODO: tune (shoot from a few distances and record the speed that scores)
        public static double[][] VELOCITY_TABLE = {
                {36, 1800},
                {120, 2400},
        };
        // Fixed speed for spinUp() and the Flywheel Tuner
        public static double TARGET_VELOCITY = 2000;  // ticks/sec. TODO: tune
        public static double VELOCITY_TOLERANCE = 50; // ticks/sec. TODO: tune
        public static double VELOCITY_NUDGE = 25;     // ticks/sec per operator press. TODO: tune
        // Velocity PIDF on the hub: F ≈ 32767 / 2800 max ticks/sec, P ≈ F / 10
        public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(1.2, 0, 0, 11.7); // TODO: tune

        // ---- Turret ----
        // Angles are robot-relative radians: 0 = facing the front of the robot, positive = counter-clockwise.
        public static double TURRET_MOTOR_TICKS_PER_REV = 145.1; // goBILDA 1150 RPM
        public static double TURRET_GEAR_RATIO = 100.0 / 30.0;   // motor revs per turret rev (30T:100T)
        // TODO: set turret soft limits from CAD
        public static double TURRET_MIN_ANGLE = -Math.PI / 2;
        public static double TURRET_MAX_ANGLE = Math.PI / 2;
        // Turret pivot relative to the robot center (inches, +x forward, +y left). TODO: set from CAD
        public static double TURRET_OFFSET_X = 0;
        public static double TURRET_OFFSET_Y = 0;
        // TODO: tune
        public static double TURRET_KP = 1.0;         // power per radian of error
        public static double TURRET_KI = 0;           // power per radian-second
        public static double TURRET_KD = 0;           // power per radian/sec of speed error
        public static double TURRET_MAX_POWER = 0.8;
        public static double TURRET_ANGLE_TOLERANCE = Math.toRadians(1);
        public static double TURRET_NUDGE = Math.toRadians(1); // per operator press

        // ---- Tracking ----
        // Velocity feedforward, so the turret keeps up while the robot drives and turns instead of lagging behind.
        // Power per radian/sec of turret speed. 1 / free speed is a good start: 1150 RPM / (100/30) ≈ 36 rad/s. TODO: tune
        public static double TURRET_KV = 1.0 / 36;
        // Aim from where the robot will be this far ahead, to make up for loop and localizer lag (seconds). TODO: tune
        public static double AIM_LOOKAHEAD = 0.05;

        // ---- Shooting on the move ----
        // A ball keeps the robot's velocity when it leaves, so lead the target by velocity × time of flight.
        public static boolean SHOOT_ON_THE_MOVE = true;
        // Ball time of flight for a distance to the hive cell: rows of {inches, seconds}, sorted by distance and
        // interpolated like VELOCITY_TABLE. Starting values are distance / horizontal speed of the slowest shot in
        // BiobuzzSim's docs/launcher-design-numbers.md (55-72° hood).
        // TODO: measure (film a few shots in slow motion; count frames from the gate to the cell)
        public static double[][] TIME_OF_FLIGHT_TABLE = {
                {30, 0.47},
                {45, 0.50},
                {60, 0.56},
                {90, 0.63},
        };
    }
}
