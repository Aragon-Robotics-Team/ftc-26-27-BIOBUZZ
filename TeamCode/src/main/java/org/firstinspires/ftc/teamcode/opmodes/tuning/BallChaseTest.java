package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_RIGHT_CORNER;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.PollenChase;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.routines.FieldPoses;
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.psilynx.psikit.core.Logger;

/**
 * Tries the pollen chase: the robot drives the intake over the pollen the ball camera sees until it has maxBalls (full),
 * or MIN_LOAD with no more pollen nearby, or nothing is in view, or timeoutS runs out. Set maxBalls and timeoutS under Configurables > BallChaseTest in Panels.
 *
 * During init pick the alliance (square / circle) and the corner the robot starts in, back against the alliance wall (dpad left /
 * right), so the chase knows where the walls, flowers and hive are. Start chases; cross chases again from wherever
 * the robot is.
 *
 * When the chase ends with pollen picked up (shootAfter), it drives to the launch spot for the up cell (FieldPoses.launch),
 * around the hive's feet and the flowers, with the flywheel spinning up and the intake still on, and shoots it all; then
 * it waits there. Chase And Shoot Test is this OpMode, but after shooting it chases again, over and over (cycles()).
 *
 * Hive vision and the turret are set up as in TeleOp: the turret stays on the hive's up cell the whole time (aimHive),
 * so the Limelight on it keeps the hive's tags in view, and hive vision keeps MatchState.hive right. Each run is logged
 * (MatchLog), and when a chase ends the telemetry says why.
 */
@Configurable
@Autonomous(name = "Ball Chase Test", group = "Tuning")
public class BallChaseTest extends OpMode {
    public static int maxBalls = RobotConstants.Intake.CAPACITY;
    public static double timeoutS = 30;
    public static boolean aimHive = true;
    public static boolean shootAfter = true;

    private Robot robot;
    private Pose corner = RED_RIGHT_CORNER;
    private PollenChase chase;
    private String stage = "not started";
    private int cycle, held, shotTotal; // chases so far; picked up and not shot yet; shot in all
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        Scheduler.reset();
        MatchLog.start(getClass().getSimpleName());
        robot = new Robot(hardwareMap);
        robot.ballVision.enable(true);
        robot.hiveVision.setEnabled(true);
        MatchState.hive = Hive.RIGHT; // new match
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, 1, telemetry);
        if (gamepad1.dpadLeftWasPressed()) corner = RED_LEFT_CORNER;
        if (gamepad1.dpadRightWasPressed()) corner = RED_RIGHT_CORNER;
        telemetry.addData("Start", "%s corner, back to the alliance wall   (dpad ←→)",
                corner == RED_LEFT_CORNER ? "our LEFT" : "our RIGHT");
        telemetry.addData("Ball vision", robot.ballVision.status());
        turretAndLimelightCheck(robot, telemetry);
        telemetry.update();
    }

    /** For init, as in Auto: whether the turret is still at its zero, and whether the Limelight sees the hive's tags. */
    static void turretAndLimelightCheck(Robot robot, Telemetry telemetry) {
        // The turret was zeroed at INIT: if it's been turned since, the aim is off by as much
        double turret = Math.toDegrees(robot.launcher.readTurretAngle());
        if (Math.abs(turret) > 2) telemetry.addLine(String.format("TURRET READS %.0f°: face it forward and INIT again", turret));
        else telemetry.addData("Turret", "%.0f°", turret);
        int tags = robot.hiveVision.tagsInView();
        if (tags > 0) telemetry.addData("Limelight", "%d hive tags in view", tags);
        else if (tags < 0) telemetry.addLine("NO LIMELIGHT IN THE CONFIGURATION");
        else if (robot.hiveVision.tagsWithout3d() > 0) {
            telemetry.addLine("Limelight sees hive tags but no 3D pose: turn on Full 3D in its pipeline");
        } else telemetry.addLine("Limelight sees no hive tags (fine if the hive is out of its view)");
    }

    @Override
    public void start() {
        robot.drivebase.setPose(MatchState.alliance.apply(corner));
        robot.start();
        if (aimHive) schedule(robot.aimAtHive());
        chaseAgain();
        loopTimer.reset();
    }

    /** After shooting, chase again (Chase And Shoot Test) instead of waiting at the launch spot. */
    protected boolean cycles() {
        return false;
    }

    private void chaseAgain() {
        schedule(cycle());
    }

    /**
     * A chase for the room left (full at maxBalls; with MIN_LOAD in all and no more pollen nearby, off to shoot), then
     * the shots, then with cycles() the next one.
     */
    private Command cycle() {
        chase = new PollenChase(robot, Math.max(1, maxBalls - held), timeoutS)
                .enoughAt(Math.max(1, RobotConstants.BallVision.MIN_LOAD - held));
        cycle++;
        stage = "chasing";
        // The intake stays on from here, to the launch spot and after: shooting goes back to intaking
        // The flywheel off while chasing (it holds its last speed otherwise), to save the battery; it spins up on the
        // way to the launch spot
        return sequential(robot.launcher.idle(), chase.command(), lazy(() -> {
            held += chase.getPickedUp();
            if (held == 0) return instant(() -> stage = "done: picked up nothing");
            if (!shootAfter) return instant(() -> stage = "done (shootAfter off)");
            return sequential(deadline(lazy(this::driveAndShoot), robot.enableFlywheel()), robot.launcher.idle(),
                    lazy(() -> held == 0 && cycles() ? cycle() : instant(() -> {})));
        }));
    }

    /** To the launch spot for the up cell, again if the hive tipped on the way, then everything into the cell. */
    private Command driveAndShoot() {
        Hive cell = MatchState.hive;
        stage = "driving to the launch spot for " + cell;
        return sequential(
                robot.driveTo(launchFor(cell)),
                lazy(() -> MatchState.hive != cell ? driveAndShoot() : sequential(
                        instant(() -> stage = "shooting into " + cell),
                        waitUntil(robot.launcher::isReady).raceWith(waitMs(2000)),
                        lazy(() -> !robot.launcher.hasShot()
                                ? instant(() -> stage = "done: no shot from the launch spot (holding the balls)")
                                : sequential(robot.startShooting(), waitMs(RobotConstants.Gate.AUTO_SHOOT_MS),
                                        robot.stopShooting(), instant(() -> {
                                            shotTotal += held;
                                            held = 0;
                                            stage = cycles() ? "shot: chasing again" : "done: shot";
                                        }))))));
    }

    /** Where to shoot into a cell from (FieldPoses.launch), for our alliance. */
    private Pose launchFor(Hive cell) {
        return new FieldPoses(PoseFactory.degrees().map(MatchState.alliance::apply)).launch(cell);
    }

    @Override
    public void loop() {
        MatchLog.beginLoop();
        robot.update();
        Scheduler.execute();
        if (gamepad1.crossWasPressed()) chaseAgain();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        telemetry.addData("Stage", "cycle %d, %s   (cross: chase again)", cycle, stage);
        telemetry.addData("Chase", chase.status());
        telemetry.addData("Balls", "holding %d, shot %d", held, shotTotal);
        telemetry.addData("Hive", "%s up   (%s)", MatchState.hive, robot.hiveVision.status());
        robot.telemetry(telemetry);
        telemetry.update();
        robot.log(loopMs);
        if (MatchLog.active()) {
            Logger.recordOutput("BallChase/stage", stage);
            Logger.recordOutput("BallChase/chase", chase.status());
            Logger.recordOutput("BallChase/pickedUp", chase.getPickedUp());
            Logger.recordOutput("BallChase/cycle", cycle);
            Logger.recordOutput("BallChase/held", held);
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
