package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.core.RobotConstants.Drive.TurnMode;

/**
 * The competition TeleOp with TWIN_STICK turning, whatever RobotConstants.Drive.TURN_MODE says: the intake turns to
 * face where the right stick points, and the robot holds that heading when it's let go. Same controls otherwise.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp (Twin Stick)", group = "Competition")
public class TeleOpTwinStick extends TeleOp {
    @Override
    protected TurnMode turnMode() {
        return TurnMode.TWIN_STICK;
    }
}
