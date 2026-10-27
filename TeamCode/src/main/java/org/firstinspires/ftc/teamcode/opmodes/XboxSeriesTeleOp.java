package org.firstinspires.ftc.teamcode.opmodes;

/**
 * TEMPORARY: TeleOp for an Xbox Series X|S controller as gamepad 1, until it's set up to report like a supported
 * controller. The Driver Station reads it as an Xbox 360, which loses the right stick and puts the triggers on the
 * right stick axes: LT arrives as right_stick_x and RT as right_stick_y, each -1 released to +1 pressed.
 *
 * Differences from TeleOp (everything else is the same):
 *   left stick x    reversed: it drove the wrong way sideways with this controller
 *   LT / RT         turn left / right (always RATE turning, no twin stick)
 *   hold circle (B) open the gate and feed the shooter (RT is taken)
 *   no slow mode    (LT is taken)
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp (Xbox Series)", group = "Competition")
public class XboxSeriesTeleOp extends TeleOp {
    private static final double DEADBAND = 0.05;

    // An axis reads 0 until it first moves, which would look like a half-pressed trigger. Each trigger only counts
    // once it has been seen at rest (-1).
    private boolean leftSeen = false, rightSeen = false;

    @Override
    protected boolean twinStick() {
        return false;
    }

    @Override
    protected double strafeInput() {
        return gamepad1.left_stick_x;
    }

    @Override
    protected double turnInput() {
        double rawLeft = gamepad1.right_stick_x, rawRight = gamepad1.right_stick_y;
        if (rawLeft < -0.5) leftSeen = true;
        if (rawRight < -0.5) rightSeen = true;
        double left = leftSeen ? (rawLeft + 1) / 2 : 0;
        double right = rightSeen ? (rawRight + 1) / 2 : 0;
        // +CCW. Which raw axis is which trigger was the other way round from the Controller Test reading, so this is
        // set from the robot: RT turns right
        double turn = right - left;
        return Math.abs(turn) < DEADBAND ? 0 : turn;
    }

    @Override
    protected boolean slowModeHeld() {
        return false;
    }

    @Override
    protected boolean shootPressed() {
        return gamepad1.circleWasPressed();
    }

    @Override
    protected boolean shootReleased() {
        return gamepad1.circleWasReleased();
    }
}
