package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/** Every robot tunable and hardware name in one place. Drivetrain / localizer config lives in pedro.Constants. */
public class RobotConstants {
    public static class Intake {
        // TODO: match the name in the robot configuration
        public static String MOTOR_NAME = "intake";

        // TODO: tune
        public static double INTAKE_POWER = 1.0;
        public static double OUTTAKE_POWER = -1.0;
    }

    public static class Launcher {
        // TODO: match the names in the robot configuration
        public static String FLYWHEEL_MOTOR_NAME = "flywheel";
        public static String TURRET_MOTOR_NAME = "turret";

        // TODO: flywheel spins to shoot on positive power; turret turns counter-clockwise (from above) on positive power
        public static DcMotorSimple.Direction FLYWHEEL_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction TURRET_DIRECTION = DcMotorSimple.Direction.FORWARD;

        // ---- Flywheel ----
        // TODO: tune
        public static double TARGET_VELOCITY = 0;     // ticks/sec
        public static double VELOCITY_TOLERANCE = 50; // ticks/sec
        // Starts at the REV hub defaults. Set F to about 32767 / (max ticks/sec), then raise P until it recovers quickly after a shot.
        public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(10, 3, 0, 0);

        // ---- Turret ----
        // Angles are robot-relative radians: 0 = facing the front of the robot, positive = counter-clockwise.
        // TODO: match the turret motor and gearing
        public static double TURRET_MOTOR_TICKS_PER_REV = 537.7; // goBILDA 312 RPM
        public static double TURRET_GEAR_RATIO = 1.0;            // motor revs per turret rev
        // TODO: set to how far the turret can turn before it hits a hard stop or wraps its wires
        public static double TURRET_MIN_ANGLE = -Math.PI / 2;
        public static double TURRET_MAX_ANGLE = Math.PI / 2;
        // TODO: tune
        public static double TURRET_KP = 0;           // power per radian of error
        public static double TURRET_KI = 0;           // power per radian-second
        public static double TURRET_KD = 0;           // power per radian/sec of turret speed
        public static double TURRET_MAX_POWER = 0.8;
        public static double TURRET_ANGLE_TOLERANCE = Math.toRadians(1);
    }
}
