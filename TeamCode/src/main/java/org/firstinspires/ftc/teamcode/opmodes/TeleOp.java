package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.RobotConstants.Drive.SLOW_MODE_SCALE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_RIGHT_CORNER;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.SHOOT_ON_THE_MOVE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Vision.HOLD_TO_LOCK_S;
import static org.firstinspires.ftc.teamcode.RobotConstants.Vision.LOCK_RUMBLE_MS;
import static org.firstinspires.ftc.teamcode.util.Html.*;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Hive;
import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.ShotSolver.Shot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.util.FieldDiagram;
import org.firstinspires.ftc.teamcode.util.HoldButton;
import org.firstinspires.ftc.teamcode.util.Html;

/**
 * Gamepad 1 (driver):
 *   sticks          field-centric drive (stick forward = away from our driver station)
 *   hold LT         slow mode
 *   triangle        toggle intake        cross   toggle outtake
 *   RB              flywheel on          LB      flywheel off
 *   hold RT         open the gate and feed the shooter
 * Gamepad 2 (operator):
 *   dpad left/right nudge turret         dpad up/down  nudge flywheel speed
 *   square / circle tap: hive LEFT / RIGHT cell is up (the Limelight keeps checking it)
 *                   hold 1 s: set that cell and lock / unlock automatic hive detection (short rumble)
 *   share / options reset pose: robot pushed into our LEFT / RIGHT corner, back to the alliance wall
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp extends OpMode {
    private static final int LEFT_COLUMN = 22; // characters, for the side-by-side numbers

    private Robot robot;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime clock = new ElapsedTime();
    private final HoldButton leftCellButton = new HoldButton(HOLD_TO_LOCK_S);
    private final HoldButton rightCellButton = new HoldButton(HOLD_TO_LOCK_S);

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        robot.hiveVision.setEnabled(true);
        // Pick up where Auto left off. Without an Auto run the robot starts at the origin with the turret
        // forward; push it into a corner and use share / options to localize.
        if (MatchState.pose != null) {
            robot.drivebase.setPose(MatchState.pose);
            robot.launcher.setCurrentTurretAngle(MatchState.turretAngle);
        }
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        telemetry.addData("Hive", MatchState.hive);
        telemetry.addData("Start pose", MatchState.pose != null ? MatchState.pose : "none (Auto didn't run)");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
        // Stick forward points away from our driver station: +x for red, -x for blue
        double headingOffset = MatchState.alliance == Alliance.BLUE ? Math.PI : 0;
        // Always-on commands, running for the whole match
        schedule(
                robot.drivebase.driveFieldCentric(
                        () -> -gamepad1.left_stick_y * driveScale(),
                        () -> -gamepad1.left_stick_x * driveScale(),
                        () -> -gamepad1.right_stick_x * driveScale(),
                        headingOffset
                ),
                robot.aimAtHive()
        );
        loopTimer.reset();
    }

    @Override
    public void loop() {
        bindControls();

        robot.update();
        Scheduler.execute();

        showTelemetry();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }

    private double driveScale() {
        return gamepad1.left_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
    }

    /** Button presses schedule commands. */
    private void bindControls() {
        // ---- Gamepad 1: driver ----
        if (gamepad1.triangleWasPressed()) schedule(robot.intake.toggleIntake());
        if (gamepad1.crossWasPressed()) schedule(robot.intake.toggleOuttake());

        if (gamepad1.rightBumperWasPressed()) schedule(robot.enableFlywheel());
        if (gamepad1.leftBumperWasPressed()) schedule(robot.launcher.idle());

        if (gamepad1.rightTriggerWasPressed()) schedule(robot.startShooting());
        if (gamepad1.rightTriggerWasReleased()) schedule(robot.stopShooting());

        // ---- Gamepad 2: operator (manual corrections) ----
        if (gamepad2.dpadLeftWasPressed()) schedule(robot.launcher.nudgeTurret(+1));
        if (gamepad2.dpadRightWasPressed()) schedule(robot.launcher.nudgeTurret(-1));
        if (gamepad2.dpadUpWasPressed()) schedule(robot.launcher.nudgeVelocity(+1));
        if (gamepad2.dpadDownWasPressed()) schedule(robot.launcher.nudgeVelocity(-1));

        hiveButton(leftCellButton.update(gamepad2.square, clock.seconds()), Hive.LEFT);
        hiveButton(rightCellButton.update(gamepad2.circle, clock.seconds()), Hive.RIGHT);

        if (gamepad2.shareWasPressed()) schedule(robot.resetPose(RED_LEFT_CORNER));
        if (gamepad2.optionsWasPressed()) schedule(robot.resetPose(RED_RIGHT_CORNER));
    }

    /** Tap: that cell is up. Hold: that cell is up, and lock or unlock automatic hive detection. */
    private void hiveButton(HoldButton.Event event, Hive side) {
        if (event == HoldButton.Event.NONE) return;
        schedule(robot.setHive(side));
        if (event == HoldButton.Event.HOLD) {
            schedule(robot.hiveVision.toggleLock());
            gamepad2.rumble(LOCK_RUMBLE_MS);
        }
    }

    /**
     * Three tiers, most important first:
     * 1. the field diagram, with the shot readouts beside it (each diagram row and its readout share a line),
     * 2. two monospace columns of numbers,
     * 3. every subsystem's own telemetry.
     * tools/telemetry-preview.html draws this layout in a browser; keep it in step.
     */
    private void showTelemetry() {
        Launcher launcher = robot.launcher;
        Shot shot = launcher.getShot();
        Pose pose = robot.drivebase.getPose();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // ---- 1. Field + what the drivers need mid-match ----
        String[] field = FieldDiagram.draw(pose, MatchState.alliance, MatchState.hive);
        String[] side = {
                reachLine(shot),
                flywheelLine(launcher),
                turretLine(launcher),
                "",
                "Alliance " + Html.alliance(MatchState.alliance),
                "Hive " + bold(color(MatchState.hive.toString(), MatchState.hive == Hive.LEFT ? PINK : CYAN)) + " " + hiveVisionWord(),
                "Intake " + intakeWord(robot.intake.getMode()) + (robot.intake.isFeeding() ? " (feeding)" : ""),
                "Gate " + (robot.gate.isOpen() ? bold(color("open", GREEN)) : bold(color("closed", GRAY))),
        };
        for (int r = 0; r < FieldDiagram.ROWS; r++) {
            telemetry.addLine(mono(field[r]) + NBSP + NBSP + (r < side.length ? side[r] : ""));
        }

        // ---- 2. Numbers, side by side ----
        if (shot != null && shot.feasible) {
            telemetry.addLine(columns(String.format("Ball speed %.0f in/s", shot.exitSpeed),
                    String.format("Flight time %.2f s", shot.timeOfFlight)));
            // How far flywheel speed and turret angle can be off and the ball still goes in
            telemetry.addLine(columns(String.format("Speed margin ±%.1f%%", shot.speedTolerance / shot.exitSpeed * 100),
                    String.format("Aim margin ±%.1f°", Math.toDegrees(shot.angleTolerance))));
            // How far off the hive the turret points to make up for the robot's motion
            telemetry.addLine(mono(String.format("Aim ahead %+.1f°", Math.toDegrees(shot.leadAngle))));
        }
        telemetry.addLine(columns(String.format("Pose %.1f, %.1f", pose.x(), pose.y()),
                String.format("Heading %.0f°", Math.toDegrees(pose.heading()))));
        telemetry.addLine(columns(String.format("Loop %.1f ms %.0f Hz", loopMs, 1000 / loopMs),
                "Follower " + robot.drivebase.getFollower().mode()));

        // ---- 3. Everything else ----
        telemetry.addLine(color("──── details ────", GRAY));
        robot.telemetry(telemetry);
        telemetry.update();
    }

    /** Two plain-text columns on one line, lined up by setting the whole line in monospace. */
    private static String columns(String left, String right) {
        StringBuilder line = new StringBuilder(left);
        for (int i = left.length(); i < LEFT_COLUMN; i++) line.append(NBSP);
        return mono(line + NBSP + right);
    }

    private static String nudge(double value, String unit) {
        return value != 0 ? String.format(" (%+.0f%s)", value, unit) : "";
    }

    /** Whether the simulated shot (on the move, if SHOOT_ON_THE_MOVE) can get the ball into the hive from here. */
    private static String reachLine(Shot shot) {
        String label = SHOOT_ON_THE_MOVE ? "SOTM " : "Shot (standing) ";
        if (shot == null) return label + color("not aiming", GRAY);
        String verdict = shot.feasible ? bold(color("can reach", GREEN)) : bold(color("can't reach", RED));
        return label + verdict + String.format(" %.0f in", shot.distance);
    }

    private String hiveVisionWord() {
        String status = robot.hiveVision.status();
        switch (status) {
            case "auto": return color("auto", GREEN);
            case "AUTO HIVE OFF": return bold(color(status, YELLOW));
            default: return color(status, GRAY);
        }
    }

    private static String intakeWord(Intake.Mode mode) {
        switch (mode) {
            case INTAKE: return bold(color("in", GREEN));
            case OUTTAKE: return bold(color("out", YELLOW));
            default: return bold(color("off", GRAY));
        }
    }

    private static String flywheelLine(Launcher launcher) {
        if (!launcher.isSpinning()) return "Flywheel " + color("off", GRAY);
        double actual = launcher.getFlywheelVelocity();
        double target = launcher.getTargetVelocity();
        String word;
        if (launcher.isFlywheelAtSpeed()) word = bold(color("ok", GREEN));
        else if (actual < target) word = bold(color("low", YELLOW));
        else word = bold(color("high", RED));
        return String.format("Flywheel %.0f/%.0f ", actual, target) + word + nudge(launcher.getVelocityNudge(), "");
    }

    private static String turretLine(Launcher launcher) {
        String word = launcher.isTurretOnTarget() ? bold(color("ok", GREEN)) : bold(color("turning", YELLOW));
        return String.format("Turret %+.0f°/%+.0f° ", Math.toDegrees(launcher.getTurretAngle()),
                Math.toDegrees(launcher.getTurretTargetAngle()))
                + word + nudge(Math.toDegrees(launcher.getTurretNudge()), "°");
    }
}
