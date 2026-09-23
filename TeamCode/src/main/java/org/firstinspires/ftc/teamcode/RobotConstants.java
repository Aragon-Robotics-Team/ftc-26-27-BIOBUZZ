package org.firstinspires.ftc.teamcode;

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
        // TODO: match the name in the robot configuration
        public static String MOTOR_NAME = "launcher";

        // TODO: tune
        public static double TARGET_VELOCITY = 0;     // ticks/sec
        public static double VELOCITY_TOLERANCE = 50; // ticks/sec
    }
}
