package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_RIGHT_CORNER;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.FieldRoutes;
import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.PollenChase;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.routines.FieldPoses;
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.psilynx.psikit.core.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * Chase-and-shoot cycles, over and over: the pollen chase until the robot has maxBalls (or a search turn finds nothing,
 * or timeoutS), then it drives to the Autos' launch spot for the cell it thinks is up (MatchState.hive, which hive
 * vision keeps right as the hive tips; see launchFor), around flowers and the hive's feet, with the turret on the hive and the
 * flywheel spinning up, shoots what it has, and chases again from there for the room left. It stops when a chase picks
 * up nothing, or when there's no shot from the launch spot. A test, not a menu Auto.
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
    private int cycle, pickedUp, held;
    private boolean fired;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        Scheduler.reset();
        MatchLog.start("ChaseAndShootTest");
        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();
        robot.ballVision.enable(true);
        robot.hiveVision.setEnabled(true);
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
        schedule(cycle());
        loopTimer.reset();
    }

    /** One chase (for the room left in the hopper), then the shots and the next cycle; or the end. */
    private Command cycle() {
        if (chase != null) pickedUp += chase.getPickedUp();
        chase = new PollenChase(robot, maxBalls - held, timeoutS);
        cycle++;
        stage = "chasing";
        return sequential(
                chase.command(),
                robot.intake.off(),
                lazy(() -> {
                    held += chase.getPickedUp();
                    if (held == 0) return instant(() -> stage = "done: the chase found no pollen");
                    return sequential(
                            deadline(lazy(this::driveAndShoot), robot.aimAtHive(), robot.enableFlywheel()),
                            lazy(() -> fired ? cycle() : instant(() -> stage = "done: no shot from the launch spot")));
                }));
    }

    /** To the launch spot for the up cell, again if the hive tipped on the way, then everything into the cell. */
    private Command driveAndShoot() {
        Hive cell = MatchState.hive;
        stage = "driving to the launch spot for " + cell;
        return sequential(
                driveTo(launchFor(cell)),
                lazy(() -> MatchState.hive != cell ? driveAndShoot() : sequential(
                        instant(() -> stage = "shooting into " + cell),
                        waitUntil(robot.launcher::isReady).raceWith(waitMs(1500)),
                        lazy(() -> {
                            fired = robot.launcher.hasShot();
                            if (!fired) return instant(() -> {});
                            held = 0;
                            return sequential(robot.startShooting(), waitMs(RobotConstants.Gate.AUTO_SHOOT_MS),
                                    robot.stopShooting());
                        }))));
    }

    /**
     * Where to shoot into a cell from: FieldPoses.startLaunch for the RIGHT (audience) cell, and that spot mirrored
     * across the hive's middle (the field's center line, y = 72 on red) for the LEFT (rear) cell.
     */
    private Pose launchFor(Hive cell) {
        Pose right = new FieldPoses(PoseFactory.degrees()).startLaunch; // red
        Pose red = cell == Hive.RIGHT ? right : new Pose(right.x(), 144 - right.y(), -right.heading());
        return MatchState.alliance.apply(red);
    }

    /** From wherever the robot is, in straight lines, around flowers and hive feet in the way (FieldRoutes). */
    private Command driveTo(Pose target) {
        List<Command> legs = new ArrayList<>();
        Pose from = follower.pose();
        for (double[] p : FieldRoutes.between(from, target)) {
            Pose via = new Pose(p[0], p[1], target.heading());
            legs.add(follow(follower, line(from, via).linear(from, via)));
            from = via;
        }
        legs.add(follow(follower, line(from, target).linear(from, target)));
        return sequential(legs.toArray(new Command[0]));
    }

    @Override
    public void loop() {
        MatchLog.beginLoop();
        robot.update();
        Scheduler.execute();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        telemetry.addData("Stage", "cycle %d, %s", cycle, stage);
        telemetry.addData("Chase", chase.status());
        telemetry.addData("Picked up", "%d in earlier cycles, holding %d", pickedUp, held);
        telemetry.addData("Hive", "%s up   (%s)", MatchState.hive, robot.hiveVision.status());
        robot.telemetry(telemetry);
        telemetry.update();
        robot.log(loopMs);
        if (MatchLog.active()) {
            Logger.recordOutput("ChaseAndShoot/cycle", cycle);
            Logger.recordOutput("ChaseAndShoot/stage", stage);
            Logger.recordOutput("ChaseAndShoot/chase", chase.status());
            Logger.recordOutput("ChaseAndShoot/pickedUp", pickedUp + chase.getPickedUp());
            Logger.recordOutput("ChaseAndShoot/held", held);
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
