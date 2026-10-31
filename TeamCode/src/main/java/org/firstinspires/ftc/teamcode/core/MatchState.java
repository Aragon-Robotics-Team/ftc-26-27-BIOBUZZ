package org.firstinspires.ftc.teamcode.core;

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

    /**
     * Call from init_loop. Square picks blue, circle red. Until one is pressed it stays what it was: what Auto ran
     * as, in TeleOp after an Auto. number is the gamepad's, for the hint.
     */
    public static void selectAlliance(Gamepad gamepad, int number, Telemetry telemetry) {
        if (gamepad.squareWasPressed()) alliance = Alliance.BLUE;
        if (gamepad.circleWasPressed()) alliance = Alliance.RED;
        telemetry.addData("Alliance", "%s   (gamepad %d: square = blue, circle = red)", alliance, number);
    }
}
