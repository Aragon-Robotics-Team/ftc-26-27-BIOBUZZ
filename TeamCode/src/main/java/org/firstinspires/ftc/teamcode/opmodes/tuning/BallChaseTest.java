package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_RIGHT_CORNER;

import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.psilynx.psikit.core.Logger;

/**
 * Tries the pollen chase: the robot drives the intake over the pollen the ball camera sees until it has maxBalls,
 * nothing is in view, or timeoutS runs out. Set maxBalls and timeoutS under Configurables > BallChaseTest in Panels.
 *
 * During init pick the alliance (square / circle) and the corner the robot starts in, back against the alliance wall (dpad left /
 * right), so the chase knows where the walls, flowers and hive are. Start chases; cross chases again from wherever
 * the robot is.
 *
 * To shoot what it picks up as well, use Chase And Shoot Test.
 *
 * Hive vision and the turret are set up as in TeleOp: the turret stays on the hive's up cell the whole time (aimHive),
 * so the Limelight on it keeps the hive's tags in view, and hive vision keeps MatchState.hive right. Each run is logged
 * (MatchLog), and when a chase ends the telemetry says why.
 */
@Configurable
@Autonomous(name = "Ball Chase Test", group = "Tuning")
public class BallChaseTest extends OpMode {
    public static int maxBalls = 4;
    public static double timeoutS = 30;
    public static boolean aimHive = true;

    private Robot robot;
    private Pose corner = RED_RIGHT_CORNER;
    private PollenChase chase;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        Scheduler.reset();
        MatchLog.start("BallChaseTest");
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

    private void chaseAgain() {
        chase = new PollenChase(robot, maxBalls, timeoutS);
        schedule(chase.command());
    }

    @Override
    public void loop() {
        MatchLog.beginLoop();
        robot.update();
        Scheduler.execute();
        if (gamepad1.crossWasPressed()) chaseAgain();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        telemetry.addData("Chase", "%s   (cross: chase again)", chase.status());
        telemetry.addData("Hive", "%s up   (%s)", MatchState.hive, robot.hiveVision.status());
        robot.telemetry(telemetry);
        telemetry.update();
        robot.log(loopMs);
        if (MatchLog.active()) {
            Logger.recordOutput("BallChase/chase", chase.status());
            Logger.recordOutput("BallChase/pickedUp", chase.getPickedUp());
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
