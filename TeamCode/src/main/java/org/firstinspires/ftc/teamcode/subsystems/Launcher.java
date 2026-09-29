package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

/**
 * Turreted flywheel shooter. The flywheel is velocity-controlled; the turret runs a PID loop on its encoder angle.
 * Flywheel commands require the flywheel motor and turret commands require the turret motor, so aiming and
 * spinning up can run at the same time.
 */
public class Launcher implements Subsystem {
    private final DcMotorEx flywheel;
    private final DcMotorEx turret;
    private final double turretTicksPerRadian = TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO / (2 * Math.PI);

    private double targetVelocity = 0;
    private double flywheelVelocity = 0;
    private double turretTargetAngle = 0;
    private double turretAngle = 0;
    private double turretAngleOffset = 0;

    private final ElapsedTime turretTimer = new ElapsedTime();
    private double turretIntegral = 0;
    private double lastTurretError = 0;
    private double lastTurretAngle = 0;

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

    /** Spin the flywheel up to TARGET_VELOCITY. Finishes right away; {@link #shoot()} waits for speed. */
    public Command spinUp() {
        return instant(() -> targetVelocity = TARGET_VELOCITY).requiring(flywheel);
    }

    /** @param velocity ticks/sec */
    public Command spinUp(double velocity) {
        // TODO: velocity from distance
        return instant(() -> targetVelocity = velocity).requiring(flywheel);
    }

    public Command idle() {
        return instant(() -> targetVelocity = 0).requiring(flywheel);
    }

    public Command toggleFlywheel() {
        return conditional(this::isSpinning, idle(), spinUp());
    }

    // ---- Turret commands ----

    /** Turn to a robot-relative angle (radians, 0 = forward, CCW positive). Finishes once on target. */
    public Command turnTurretTo(double angle) {
        return Command.build()
                .setStart(() -> setTurretTarget(angle))
                .setDone(this::isTurretOnTarget)
                .requiring(turret);
    }

    /** Keep the turret pointed at a field position. Runs until interrupted. */
    public Command aimAt(Supplier<Pose> robotPose, Pose target) {
        return infinite(() -> {
            // TODO: turret offset from center
            Pose pose = robotPose.get();
            double fieldAngle = Math.atan2(target.y() - pose.y(), target.x() - pose.x());
            setTurretTarget(fieldAngle - pose.heading());
        }).requiring(turret);
    }

    // ---- Shooting commands ----

    /** Wait until ready (max 1.5 s so a slow flywheel can't stall), then fire. */
    public Command shoot() {
        // TODO: require the feeder once it exists
        return sequential(
                waitUntil(this::isReady).raceWith(waitMs(1500)),
                instant(this::fire),
                waitMs(250) // TODO: tune shot delay
        );
    }

    private void fire() {
        // TODO: fire feeder / gate
    }

    // ---- State ----

    public boolean isSpinning() {
        return targetVelocity != 0;
    }

    public boolean isFlywheelAtSpeed() {
        return isSpinning() && Math.abs(targetVelocity - flywheelVelocity) <= VELOCITY_TOLERANCE;
    }

    /** Robot-relative turret angle in radians, as of the last {@link #update()}. */
    public double getTurretAngle() {
        return turretAngle;
    }

    public boolean isTurretOnTarget() {
        return Math.abs(turretTargetAngle - getTurretAngle()) <= TURRET_ANGLE_TOLERANCE;
    }

    /** Flywheel is at speed and the turret is on target. */
    public boolean isReady() {
        return isFlywheelAtSpeed() && isTurretOnTarget();
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

    /** Clamped to [TURRET_MIN_ANGLE, TURRET_MAX_ANGLE], so targets past the limits need the drivetrain to turn. */
    private void setTurretTarget(double angle) {
        turretTargetAngle = Range.clip(Angle.normalizeSigned(angle), TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
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
        // Derivative on measurement, so a new target doesn't kick the output
        double velocity = (turretAngle - lastTurretAngle) / dt;
        lastTurretError = error;
        lastTurretAngle = turretAngle;

        double power = TURRET_KP * error + TURRET_KI * turretIntegral - TURRET_KD * velocity;
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
    }
}
