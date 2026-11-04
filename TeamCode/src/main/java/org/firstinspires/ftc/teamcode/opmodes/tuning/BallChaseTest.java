package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_RIGHT_CORNER;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.PollenChase;
import org.firstinspires.ftc.teamcode.core.Robot;

/**
 * Tries the pollen chase: the robot drives the intake over the pollen the ball camera sees until it has maxBalls,
 * nothing is in view, or timeoutS runs out. Set maxBalls and timeoutS under Configurables > BallChaseTest in Panels.
 *
 * During init pick the alliance (square / circle) and the corner the robot starts in, back against the alliance wall (dpad left /
 * right), so the chase knows where the walls, flowers and hive are. Start chases; cross chases again from wherever
 * the robot is.
 */
@Configurable
@Autonomous(name = "Ball Chase Test", group = "Tuning")
public class BallChaseTest extends OpMode {
    public static int maxBalls = 4;
    public static double timeoutS = 30;

    private Robot robot;
    private Pose corner = RED_LEFT_CORNER;
    private PollenChase chase;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(hardwareMap);
        robot.ballVision.enable(true);
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, 1, telemetry);
        if (gamepad1.dpadLeftWasPressed()) corner = RED_LEFT_CORNER;
        if (gamepad1.dpadRightWasPressed()) corner = RED_RIGHT_CORNER;
        telemetry.addData("Start", "%s corner, back to the alliance wall   (dpad ←→)",
                corner == RED_LEFT_CORNER ? "our LEFT" : "our RIGHT");
        telemetry.addData("Ball vision", robot.ballVision.status());
        telemetry.update();
    }

    @Override
    public void start() {
        robot.drivebase.setPose(MatchState.alliance.apply(corner));
        robot.start();
        chaseAgain();
    }

    private void chaseAgain() {
        chase = new PollenChase(robot, maxBalls, timeoutS);
        schedule(chase.command());
    }

    @Override
    public void loop() {
        robot.update();
        Scheduler.execute();
        if (gamepad1.crossWasPressed()) chaseAgain();

        telemetry.addData("Chase", "%s   (cross: chase again)", chase.status());
        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }
}
