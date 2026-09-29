package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ShotSolver;
import org.firstinspires.ftc.teamcode.ShotSolver.Shot;

import java.util.function.Supplier;

/**
 * Turreted flywheel shooter. The flywheel is velocity-controlled; the turret runs a PID loop on its encoder angle,
 * plus a velocity feedforward so it can track a target while the robot moves.
 * Aiming simulates every shot ({@link ShotSolver}): the turret heading and flywheel speed come from a flight model
 * with drag that allows for the robot's motion, and "ready" means both are within the error that shot can take.
 * Flywheel commands require the flywheel motor and turret commands require the turret motor, so aiming and
 * spinning up can run at the same time. The gate and intake do the actual feeding (see Robot).
 */
public class Launcher implements Subsystem {
    /** What the launcher is waiting on. The LED shows this. */
    public enum Status {
        OFF,         // flywheel off
        NO_SHOT,     // no flywheel speed reaches the target from here
        SPINNING_UP, // flywheel not at speed yet (spinning up, or settling to a new target)
        AIMING,      // flywheel at speed, turret not on target yet
        READY
    }

    private final DcMotorEx flywheel;
    private final DcMotorEx turret;
    private final double turretTicksPerRadian = TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO / (2 * Math.PI);

    private double targetVelocity = 0;
    private double flywheelVelocity = 0;
    private double turretTargetAngle = 0;
    private double turretTargetRate = 0; // rad/s the target is moving at, for the feedforward
    private double turretAngle = 0;
    private double turretAngleOffset = 0;

    // Operator corrections for drift in the velocity table or localization. New every OpMode, so they reset each match.
    private double velocityNudge = 0;
    private double turretNudge = 0;

    private final ElapsedTime turretTimer = new ElapsedTime();
    private double turretIntegral = 0;
    private double lastTurretError = 0;
    private double lastTurretAngle = 0;
    private double turretVelocity = 0; // rad/s, measured

    private final ShotSolver shotSolver = new ShotSolver();
    // The shots the flywheel and turret are following, while enableFlywheel() / aimAt() run
    private Shot flywheelShot = null;
    private Shot turretShot = null;

    public Launcher(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheel.setDirection(FLYWHEEL_DIRECTION);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, don't brake the flywheel
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, FLYWHEEL_PIDF);

        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
        turret.setDirection(TURRET_DIRECTION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Zeroes the turret wherever it is at init, so it must start facing forward
        // unless the OpMode calls setCurrentTurretAngle() (e.g. TeleOp after Auto).
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ---- Flywheel commands ----

    /**
     * Keep the flywheel at the speed the simulated shot at the target needs, plus the operator's nudge.
     * Runs until {@link #idle()}.
     *
     * @param target center of a hive cell's opening, heading pointing out of it
     */
    public Command enableFlywheel(Supplier<Pose> robotPose, Supplier<Velocity> robotVelocity, Supplier<Pose> target) {
        return infinite(() -> {
            flywheelShot = shotSolver.solve(robotPose.get(), robotVelocity.get(), target.get());
            targetVelocity = flywheelShot.flywheelVelocity + velocityNudge;
        }).setEnd(end -> flywheelShot = null).requiring(flywheel);
    }

    /** Spin the flywheel at the fixed TARGET_VELOCITY. Used by the Flywheel Tuner. */
    public Command spinUp() {
        return instant(() -> targetVelocity = TARGET_VELOCITY).requiring(flywheel);
    }

    public Command idle() {
        return instant(() -> targetVelocity = 0).requiring(flywheel);
    }

    public Command toggleFlywheel() {
        return conditional(this::isSpinning, idle(), spinUp());
    }

    /** @param direction +1 faster, -1 slower */
    public Command nudgeVelocity(int direction) {
        return instant(() -> velocityNudge += direction * VELOCITY_NUDGE);
    }

    // ---- Turret commands ----

    /** Turn to a robot-relative angle (radians, 0 = forward, CCW positive). Finishes once on target. */
    public Command turnTurretTo(double angle) {
        return Command.build()
                .setStart(() -> setTurretTarget(angle, 0))
                .setDone(this::isTurretOnTarget)
                .requiring(turret);
    }

    /**
     * Keep the turret on the heading the simulated shot at the target needs (leading it while the robot moves), plus
     * the operator's nudge. Runs until interrupted.
     *
     * @param target center of a hive cell's opening, heading pointing out of it
     */
    public Command aimAt(Supplier<Pose> robotPose, Supplier<Velocity> robotVelocity, Supplier<Pose> target) {
        return infinite(() -> {
            turretShot = shotSolver.solve(robotPose.get(), robotVelocity.get(), target.get());
            setTurretTarget(turretShot.turretAngle + turretNudge, turretShot.turretRate);
        }).setEnd(end -> {
            // Hold the last angle; a leftover feedforward would drag the turret off it
            turretTargetRate = 0;
            turretShot = null;
        }).requiring(turret);
    }

    /** @param direction +1 left (CCW), -1 right */
    public Command nudgeTurret(int direction) {
        return instant(() -> turretNudge += direction * TURRET_NUDGE);
    }

    // ---- Aiming math ----

    /** Inches from the turret pivot to a field position. */
    public double distanceTo(Pose robotPose, Pose target) {
        Pose pivot = ShotSolver.turretPivot(robotPose);
        return Math.hypot(target.x() - pivot.x(), target.y() - pivot.y());
    }

    // ---- State ----

    public boolean isSpinning() {
        return targetVelocity != 0;
    }

    /** Within the speed error the current shot can take (VELOCITY_TOLERANCE when not aiming at a target). */
    public boolean isFlywheelAtSpeed() {
        return isSpinning() && Math.abs(targetVelocity - flywheelVelocity) <= flywheelTolerance();
    }

    private double flywheelTolerance() {
        if (flywheelShot == null || !flywheelShot.feasible) return VELOCITY_TOLERANCE;
        return flywheelShot.speedTolerance * FLYWHEEL_TICKS_PER_EXIT_SPEED * READY_WINDOW_FRACTION;
    }

    public double getFlywheelVelocity() {
        return flywheelVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocityNudge() {
        return velocityNudge;
    }

    /** Robot-relative turret angle in radians, as of the last {@link #update()}. */
    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTurretTargetAngle() {
        return turretTargetAngle;
    }

    public double getTurretNudge() {
        return turretNudge;
    }

    /** Within the angle error the current shot can take (TURRET_ANGLE_TOLERANCE when not aiming at a target). */
    public boolean isTurretOnTarget() {
        return Math.abs(turretTargetAngle - getTurretAngle()) <= turretTolerance();
    }

    private double turretTolerance() {
        if (turretShot == null || !turretShot.feasible) return TURRET_ANGLE_TOLERANCE;
        return turretShot.angleTolerance * READY_WINDOW_FRACTION;
    }

    /** False when aiming at a target that no flywheel speed can reach from here (e.g. too close for the hood). */
    public boolean hasShot() {
        return (flywheelShot == null || flywheelShot.feasible) && (turretShot == null || turretShot.feasible);
    }

    /** There's a shot, the flywheel is at speed and the turret is on target. */
    public boolean isReady() {
        return hasShot() && isFlywheelAtSpeed() && isTurretOnTarget();
    }

    public Status getStatus() {
        if (!isSpinning()) return Status.OFF;
        if (isReady()) return Status.READY;
        if (!hasShot()) return Status.NO_SHOT;
        return isFlywheelAtSpeed() ? Status.AIMING : Status.SPINNING_UP;
    }

    /** The shot the turret (else the flywheel) is following, or null when neither is aiming at a target. */
    public Shot getShot() {
        return turretShot != null ? turretShot : flywheelShot;
    }

    // ---- Setup ----

    /** Push new velocity PIDF to the motor controller. Used by the tuning OpMode. */
    public void setFlywheelPIDF(PIDFCoefficients pidf) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    /** Tell the launcher where the turret is right now, e.g. the angle Auto left it at. */
    public void setCurrentTurretAngle(double angle) {
        turretAngleOffset = angle - turret.getCurrentPosition() / turretTicksPerRadian;
        turretAngle = angle;
        lastTurretAngle = angle;
    }

    /**
     * Clamped to [TURRET_MIN_ANGLE, TURRET_MAX_ANGLE], so targets past the limits need the drivetrain to turn.
     * @param rate how fast the target is moving (radians/sec), for the feedforward
     */
    private void setTurretTarget(double angle, double rate) {
        double normalized = Angle.normalizeSigned(angle);
        turretTargetAngle = Range.clip(normalized, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
        // Parked on a soft limit: don't push into it
        turretTargetRate = turretTargetAngle == normalized ? rate : 0;
    }

    @Override
    public void start() {
        turretTimer.reset();
        turretIntegral = 0;
        lastTurretError = 0;
        lastTurretAngle = turretAngle;
    }

    @Override
    public void update() {
        flywheelVelocity = flywheel.getVelocity();
        turretAngle = turret.getCurrentPosition() / turretTicksPerRadian + turretAngleOffset;
        if (isSpinning()) flywheel.setVelocity(targetVelocity);
        else flywheel.setPower(0);

        updateTurret();
    }

    private void updateTurret() {
        double dt = turretTimer.seconds();
        turretTimer.reset();
        if (dt <= 0) return;

        double error = turretTargetAngle - turretAngle;
        // Anti-windup: dump the integral when the error crosses zero
        if (Math.signum(error) != Math.signum(lastTurretError)) turretIntegral = 0;
        turretIntegral += error * dt;
        // Derivative of the error from the measured speed and the target's known rate (not by differencing the
        // error), so a new target doesn't kick the output
        turretVelocity = (turretAngle - lastTurretAngle) / dt;
        lastTurretError = error;
        lastTurretAngle = turretAngle;

        double power = TURRET_KP * error + TURRET_KI * turretIntegral + TURRET_KD * (turretTargetRate - turretVelocity)
                + TURRET_KV * turretTargetRate;
        turret.setPower(Range.clip(power, -TURRET_MAX_POWER, TURRET_MAX_POWER));
    }

    @Override
    public void stop() {
        targetVelocity = 0;
        flywheel.setPower(0);
        turret.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Launcher ready", isReady());
        telemetry.addData("Flywheel target / actual", "%.0f / %.0f", targetVelocity, flywheelVelocity);
        telemetry.addData("Turret target / actual (deg)", "%.1f / %.1f",
                Math.toDegrees(turretTargetAngle), Math.toDegrees(getTurretAngle()));
        telemetry.addData("Turret turn speed target / actual (deg/s)", "%.0f / %.0f",
                Math.toDegrees(turretTargetRate), Math.toDegrees(turretVelocity));
        telemetry.addData("Turret power", "%.2f", turret.getPower());
        Shot shot = getShot();
        if (shot == null) return;
        if (!shot.feasible) {
            telemetry.addData("Planned shot", "none from here (%.0f in)", shot.distance);
            return;
        }
        telemetry.addData("Planned shot", "ball %.0f in/s, flight %.2f s, aim ahead %+.1f°, %.0f in",
                shot.exitSpeed, shot.timeOfFlight, Math.toDegrees(shot.leadAngle), shot.distance);
        telemetry.addData("Shot margins", "speed ±%.1f%%, aim ±%.1f°",
                shot.speedTolerance / shot.exitSpeed * 100, Math.toDegrees(shot.angleTolerance));
        if (flywheelShot != null && flywheelShot.feasible) {
            // With the nudge set so standing shots go through the middle, this is FLYWHEEL_TICKS_PER_EXIT_SPEED
            telemetry.addData("Calibration (ticks/sec per in/s)", "%.2f", targetVelocity / flywheelShot.exitSpeed);
        }
    }
}
