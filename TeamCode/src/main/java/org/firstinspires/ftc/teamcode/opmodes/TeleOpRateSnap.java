package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.core.RobotConstants.Drive.TurnMode;

/**
 * The competition TeleOp with RATE_SNAP turning, whatever RobotConstants.Drive.TURN_MODE says: right stick x sets the
 * turn speed, and driving hard with it centered lines the robot up with the drive direction. Same controls otherwise.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp (Rate Snap)", group = "Competition")
public class TeleOpRateSnap extends TeleOp {
    @Override
    protected TurnMode turnMode() {
        return TurnMode.RATE_SNAP;
    }
}
