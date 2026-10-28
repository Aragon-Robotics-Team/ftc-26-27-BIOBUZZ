package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Drive.SLOW_MODE_SCALE;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_RIGHT_CORNER;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.ShotSolver;
import org.firstinspires.ftc.teamcode.core.ShotSolver.Shot;

import java.util.ArrayList;
import java.util.List;

/**
 * Calibrate the shot model from real shots. ShotSolver works out the ball exit speed for each spot; this finds the
 * two numbers that turn it into the right flywheel speed:
 * - FLYWHEEL_TICKS_PER_EXIT_SPEED: flywheel ticks/sec per inch/sec of ball speed.
 * - DRAG_COEFFICIENT: if far shots need a different ticks / exit speed than near ones, the model's drag is off.
 *   Too little drag in the model shows up as the ratio rising with distance.
 *
 * <p>Localize first (gamepad 2 share / options with the robot pushed into a corner), then for each spot:
 * 1. Drive there and stop, facing roughly at the hive. Flywheel on (gamepad 1 RB); turret and flywheel aim at the
 *    up cell for a standing shot.
 * 2. Shoot (hold gamepad 1 RT) and nudge the flywheel (gamepad 2 dpad up / down) until balls go through the middle.
 * 3. Gamepad 2 cross saves the spot. Gamepad 2 triangle deletes the last one.
 * Save 5 or more spots spread from the closest shot to the farthest. The results are recomputed on every save:
 * paste the "Fitted" lines into RobotConstants.Launcher. Saved spots last until the Robot Controller app restarts,
 * so the OpMode can be stopped and run again; check the fit by running it again with the new constants
 * (the ratio should then stay flat with distance).
 *
 * <p>Gamepad 1 drives like TeleOp (left stick field-centric, right stick x turns, hold LT slow), triangle /
 * cross toggle intake / outtake. Gamepad 2 dpad left / right nudge the turret, square / circle pick the LEFT / RIGHT
 * cell. Alliance at init: gamepad 1 X / B.
 */
@Configurable
@TeleOp(name = "Shot Calibration", group = "Tuning")
public class ShotCalibration extends OpMode {
    // Saving needs the robot standing still: the model is for a standing shot
    public static double stillSpeed = 2;              // in/s
    public static double stillTurnDegPerSec = 5;
    // Drag coefficients to try in the fit
    public static double dragMin = 0;
    public static double dragMax = 1.5;
    public static double dragStep = 0.05;
    // The fit needs this many spots, spread at least this far apart (inches)
    public static int fitMinSamples = 3;
    public static double fitMinSpan = 24;

    /** One saved spot: the flywheel speed that put standing shots through the middle of the opening. */
    static class Sample {
        final Pose robotPose;
        final Pose target;
        final double distance;
        final double ticks; // flywheel ticks/sec

        Sample(Pose robotPose, Pose target, double distance, double ticks) {
            this.robotPose = robotPose;
            this.target = target;
            this.distance = distance;
            this.ticks = ticks;
        }
    }

    /** Ticks / exit speed over the samples, at one drag coefficient. */
    static class Result {
        final double drag;
        final double[] ratios;       // per sample, in order
        final double meanRatio;
        final double spread;         // standard deviation / mean
        final double changeOverRange; // fitted change from the nearest to the farthest sample, fraction of the mean

        Result(double drag, double[] ratios, double meanRatio, double spread, double changeOverRange) {
            this.drag = drag;
            this.ratios = ratios;
            this.meanRatio = meanRatio;
            this.spread = spread;
            this.changeOverRange = changeOverRange;
        }
    }

    // Static so the saved spots survive stopping and re-running the OpMode
    private static final List<Sample> samples = new ArrayList<>();

    private Robot robot;
    private TelemetryManager panels;
    private Result current; // at the DRAG_COEFFICIENT in RobotConstants
    private Result fitted;  // at the best drag, or null
    private String message = "";

    @Override
    public void init() {
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        if (MatchState.pose != null) {
            robot.drivebase.setPose(MatchState.pose);
            robot.launcher.setCurrentTurretAngle(MatchState.turretAngle);
        }
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        telemetry.addData("Saved spots", samples.size());
        telemetry.addLine("Turret must face forward at init. Localize after start: gamepad 2 share / options in a corner");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
        double headingOffset = MatchState.alliance == Alliance.BLUE ? Math.PI : 0;
        // Standing shots: zero velocity for both, so they share one solve per loop
        schedule(
                robot.drivebase.driveFieldCentric(
                        () -> -gamepad1.left_stick_y * driveScale(),
                        () -> -gamepad1.left_stick_x * driveScale(),
                        () -> -gamepad1.right_stick_x * driveScale(),
                        headingOffset),
                robot.launcher.aimAt(robot.drivebase::getPose, Velocity::zero, robot::hiveTarget)
        );
        analyze();
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

    private void bindControls() {
        // ---- Gamepad 1: same as TeleOp ----
        if (gamepad1.triangleWasPressed()) schedule(robot.intake.toggleIntake());
        if (gamepad1.crossWasPressed()) schedule(robot.intake.toggleOuttake());

        if (gamepad1.rightBumperWasPressed()) {
            schedule(robot.launcher.enableFlywheel(robot.drivebase::getPose, Velocity::zero, robot::hiveTarget));
        }
        if (gamepad1.leftBumperWasPressed()) schedule(robot.launcher.idle());

        if (gamepad1.rightTriggerWasPressed()) schedule(robot.startShooting());
        if (gamepad1.rightTriggerWasReleased()) schedule(robot.stopShooting());

        // ---- Gamepad 2: nudges and saving ----
        if (gamepad2.dpadLeftWasPressed()) schedule(robot.launcher.nudgeTurret(+1));
        if (gamepad2.dpadRightWasPressed()) schedule(robot.launcher.nudgeTurret(-1));
        if (gamepad2.dpadUpWasPressed()) schedule(robot.launcher.nudgeVelocity(+1));
        if (gamepad2.dpadDownWasPressed()) schedule(robot.launcher.nudgeVelocity(-1));

        if (gamepad2.squareWasPressed()) schedule(robot.setHive(Hive.LEFT));
        if (gamepad2.circleWasPressed()) schedule(robot.setHive(Hive.RIGHT));

        if (gamepad2.shareWasPressed()) schedule(robot.resetPose(RED_LEFT_CORNER));
        if (gamepad2.optionsWasPressed()) schedule(robot.resetPose(RED_RIGHT_CORNER));

        if (gamepad2.crossWasPressed()) save();
        if (gamepad2.triangleWasPressed() && !samples.isEmpty()) {
            samples.remove(samples.size() - 1);
            message = "Deleted the last spot";
            analyze();
        }
    }

    /** Why this spot can't be saved right now, or null if it can. */
    private String cantSave() {
        Shot shot = robot.launcher.getShot();
        if (!robot.launcher.isSpinning()) return "flywheel is off (gamepad 1 RB)";
        if (shot == null || !shot.feasible) return "the model has no shot from here";
        Velocity velocity = robot.drivebase.getVelocity();
        if (Math.hypot(velocity.vx, velocity.vy) > stillSpeed
                || Math.abs(velocity.omega) > Math.toRadians(stillTurnDegPerSec)) return "robot is moving";
        return null;
    }

    private void save() {
        String reason = cantSave();
        if (reason != null) {
            message = "Not saved: " + reason;
            return;
        }
        Pose pose = robot.drivebase.getPose();
        Pose target = robot.hiveTarget();
        samples.add(new Sample(pose, target, robot.launcher.distanceTo(pose, target),
                robot.launcher.getTargetVelocity()));
        message = String.format("Saved spot %d", samples.size());
        analyze();
    }

    /** Recompute the results. The fit re-solves every spot many times, so only on a change, not every loop. */
    private void analyze() {
        current = evaluate(samples, RobotConstants.Launcher.DRAG_COEFFICIENT);
        fitted = canFit(samples) ? fitDrag(samples) : null;
    }

    private void showTelemetry() {
        Shot shot = robot.launcher.getShot();
        Pose pose = robot.drivebase.getPose();

        panels.addLine("== SHOT CALIBRATION ==");
        panels.addLine("G1: drive, RB/LB flywheel on/off, hold RT shoot. G2: dpad up/down speed, cross save, triangle delete last");
        panels.addData("Alliance / cell", MatchState.alliance + " / " + MatchState.hive + "   (G2 square / circle)");
        panels.addData("Pose", String.format("%.1f, %.1f, %.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        panels.addData("Distance (in)", String.format("%.1f", robot.distanceToHive()));
        if (shot != null && shot.feasible) {
            panels.addData("Model ball speed (in/s)", String.format("%.0f   margin ±%.1f%%",
                    shot.exitSpeed, shot.speedTolerance / shot.exitSpeed * 100));
        } else {
            panels.addData("Model ball speed (in/s)", "no shot from here");
        }
        panels.addData("Flywheel target / actual", String.format("%.0f / %.0f   nudge %+.0f",
                robot.launcher.getTargetVelocity(), robot.launcher.getFlywheelVelocity(),
                robot.launcher.getVelocityNudge()));
        if (robot.launcher.isSpinning() && shot != null && shot.feasible) {
            panels.addData("Ticks / ball speed here", String.format("%.2f",
                    robot.launcher.getTargetVelocity() / shot.exitSpeed));
        }
        String reason = cantSave();
        panels.addData("Save (G2 cross)",reason == null ? "ready" : reason);
        if (!message.isEmpty()) panels.addLine(message);

        panels.addLine("");
        panels.addLine(String.format("== %d SAVED SPOTS (ticks / ball speed at drag %.2f) ==",
                samples.size(), RobotConstants.Launcher.DRAG_COEFFICIENT));
        for (int i = 0; i < samples.size(); i++) {
            Sample sample = samples.get(i);
            String ratio = current != null && i < current.ratios.length && !Double.isNaN(current.ratios[i])
                    ? String.format("%.2f", current.ratios[i]) : "no shot";
            panels.addLine(String.format("%d: %.0f in   %.0f ticks/s   %s", i + 1, sample.distance, sample.ticks, ratio));
        }
        if (current != null && !Double.isNaN(current.meanRatio)) {
            panels.addData("Average", String.format("%.2f (spread ±%.1f%%)", current.meanRatio, current.spread * 100));
            if (canFit(samples)) {
                panels.addData("Change near → far", String.format("%+.1f%%   %s",
                        current.changeOverRange * 100, driftAdvice(current.changeOverRange)));
            }
        }

        panels.addLine("");
        if (fitted != null) {
            panels.addLine(String.format("== FITTED (spread ±%.1f%%, change near → far %+.1f%%) ==",
                    fitted.spread * 100, fitted.changeOverRange * 100));
            panels.addLine(String.format("DRAG_COEFFICIENT = %.2f;", fitted.drag));
            panels.addLine(String.format("FLYWHEEL_TICKS_PER_EXIT_SPEED = %.2f;", fitted.meanRatio));
            if (fitted.drag <= dragMin + 1e-9 || fitted.drag >= dragMax - 1e-9) {
                panels.addLine("Drag hit the end of the search range: check the spots, or try another LIFT_COEFFICIENT");
            }
        } else if (!samples.isEmpty()) {
            panels.addLine(String.format("Fit needs %d+ spots at least %.0f in apart", fitMinSamples, fitMinSpan));
        }
        panels.update(telemetry);
    }

    private static String driftAdvice(double change) {
        if (Math.abs(change) < 0.02) return "(flat: model OK)";
        return change > 0 ? "(far shots need more: more drag)" : "(far shots need less: less drag)";
    }

    // ---- Fitting (no hardware, so it can be unit tested) ----

    static boolean canFit(List<Sample> samples) {
        if (samples.size() < fitMinSamples) return false;
        double near = Double.MAX_VALUE, far = 0;
        for (Sample sample : samples) {
            near = Math.min(near, sample.distance);
            far = Math.max(far, sample.distance);
        }
        return far - near >= fitMinSpan;
    }

    /** The drag coefficient in [dragMin, dragMax] that makes ticks / exit speed most nearly the same at every spot. */
    static Result fitDrag(List<Sample> samples) {
        Result best = null;
        for (int i = 0; dragMin + i * dragStep <= dragMax + 1e-9; i++) {
            Result result = evaluate(samples, dragMin + i * dragStep);
            if (Double.isNaN(result.spread)) continue; // some spot has no shot at this drag
            if (best == null || result.spread < best.spread) best = result;
        }
        return best;
    }

    /**
     * Re-solve every spot with this drag coefficient and compare the saved flywheel speeds with the model's exit
     * speeds. A spot the model can't reach gets a NaN ratio, and makes the mean, spread and change NaN.
     */
    static Result evaluate(List<Sample> samples, double drag) {
        double savedDrag = RobotConstants.Launcher.DRAG_COEFFICIENT;
        double savedMax = RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY;
        double[] ratios = new double[samples.size()];
        try {
            RobotConstants.Launcher.DRAG_COEFFICIENT = drag;
            // The solver calls a shot infeasible above FLYWHEEL_MAX_VELOCITY / FLYWHEEL_TICKS_PER_EXIT_SPEED, and that
            // ratio is what's being calibrated: lift the cap so a wrong ratio doesn't rule out a shot that went in
            RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY = savedMax * 3;
            for (int i = 0; i < samples.size(); i++) {
                Sample sample = samples.get(i);
                Shot shot = new ShotSolver().solve(sample.robotPose, Velocity.zero(), sample.target);
                ratios[i] = shot.feasible ? sample.ticks / shot.exitSpeed : Double.NaN;
            }
        } finally {
            RobotConstants.Launcher.DRAG_COEFFICIENT = savedDrag;
            RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY = savedMax;
        }

        double sum = 0;
        for (double ratio : ratios) sum += ratio;
        double mean = sum / ratios.length;
        double squares = 0;
        for (double ratio : ratios) squares += (ratio - mean) * (ratio - mean);
        double spread = Math.sqrt(squares / ratios.length) / mean;

        // Least-squares line through ratio vs distance, as the change over the range of distances
        double meanDistance = 0;
        for (Sample sample : samples) meanDistance += sample.distance / samples.size();
        double sxy = 0, sxx = 0, near = Double.MAX_VALUE, far = 0;
        for (int i = 0; i < samples.size(); i++) {
            double dx = samples.get(i).distance - meanDistance;
            sxy += dx * (ratios[i] - mean);
            sxx += dx * dx;
            near = Math.min(near, samples.get(i).distance);
            far = Math.max(far, samples.get(i).distance);
        }
        double change = sxx > 0 ? sxy / sxx * (far - near) / mean : 0;
        return new Result(drag, ratios, mean, spread, change);
    }
}
