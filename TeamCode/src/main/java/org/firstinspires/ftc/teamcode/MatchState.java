package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Carries state from Auto into TeleOp. Static fields survive between OpModes until the robot app restarts. */
public class MatchState {
    public static Alliance alliance = Alliance.RED;
    /** Which of our hive's cells is up. Auto resets it to RIGHT at the start of a match. */
    public static Hive hive = Hive.RIGHT;

    /** Where Auto left the robot, or null if Auto hasn't run. */
    public static Pose pose = null;
    public static double turretAngle = 0;

    /** Call from init_loop. X picks blue, B picks red (square / circle on PlayStation). */
    public static void selectAlliance(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.xWasPressed()) alliance = Alliance.BLUE;
        if (gamepad.bWasPressed()) alliance = Alliance.RED;
        telemetry.addData("Alliance", "%s   (X = blue, B = red)", alliance);
    }
}
