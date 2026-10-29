package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_RIGHT_CORNER;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.PollenChase;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.routines.FieldPoses;
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.psilynx.psikit.core.Logger;

/**
 * Tries a whole chase cycle: the pollen chase until the robot has maxBalls (or nothing is left in view, or timeoutS),
 * then it drives to the Autos' launch spot (FieldPoses.startLaunch) with the turret on the hive and the flywheel
 * spinning up, and shoots everything into the up cell. A test, not a menu Auto.
 *
 * During init pick the alliance (X / B) and the corner the robot starts in, back against the alliance wall (dpad left /
 * right). The hive starts with the RIGHT cell up, as at the start of a match.
 */
@Configurable
@Autonomous(name = "Chase And Shoot Test", group = "Tuning")
public class ChaseAndShootTest extends OpMode {
    public static int maxBalls = 4;
    public static double timeoutS = 25;

    private Robot robot;
    private Follower follower;
    private Pose corner = RED_LEFT_CORNER;
    private PollenChase chase;
    private String stage = "not started";
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        Scheduler.reset();
        MatchLog.start("ChaseAndShootTest");
        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();
        robot.ballVision.enable(true);
        MatchState.hive = Hive.RIGHT; // new match
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
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
        Pose launch = new FieldPoses(PoseFactory.degrees().map(MatchState.alliance::apply)).startLaunch;
        chase = new PollenChase(robot, maxBalls, timeoutS);
        schedule(sequential(
                instant(() -> stage = "chasing"),
                chase.command(),
                robot.intake.off(),
                instant(() -> stage = "driving to the launch spot"),
                deadline(
                        sequential(
                                // From wherever the chase ended, so the path is made when this step starts
                                lazy(() -> follow(follower, line(follower.pose(), launch).linear(follower.pose(), launch))),
                                instant(() -> stage = "shooting"),
                                robot.shoot()),
                        robot.aimAtHive(), robot.enableFlywheel()),
                instant(() -> stage = "done")));
        loopTimer.reset();
    }

    @Override
    public void loop() {
        MatchLog.beginLoop();
        robot.update();
        Scheduler.execute();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        telemetry.addData("Stage", stage);
        telemetry.addData("Chase", chase.status());
        robot.telemetry(telemetry);
        telemetry.update();
        robot.log(loopMs);
        if (MatchLog.active()) {
            Logger.recordOutput("ChaseAndShoot/stage", stage);
            Logger.recordOutput("ChaseAndShoot/pickedUp", chase.getPickedUp());
        }
        MatchLog.endLoop();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
        MatchLog.stop();
    }
}
