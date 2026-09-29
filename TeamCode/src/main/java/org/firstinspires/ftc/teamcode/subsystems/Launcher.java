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

import java.util.function.Supplier;

/**
 * Turreted flywheel shooter. The flywheel is velocity-controlled; the turret runs a PID loop on its encoder angle,
 * plus a velocity feedforward so it can track a target while the robot moves.
 * Aiming allows for the robot's motion (see {@link #solve}): the ball keeps the robot's velocity, so while driving
 * the turret leads the target and the flywheel is set for the distance to that lead point.
 * Flywheel commands require the flywheel motor and turret commands require the turret motor, so aiming and
 * spinning up can run at the same time. The gate and intake do the actual feeding (see Robot).
 */
public class Launcher implements Subsystem {
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

    private Shot lastShot = null; // for telemetry

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
     * Keep the flywheel at the speed the velocity table gives for the distance to the target (led for the robot's
     * motion), plus the operator's nudge. Runs until {@link #idle()}.
     */
    public Command enableFlywheel(Supplier<Pose> robotPose, Supplier<Velocity> robotVelocity, Supplier<Pose> target) {
        return infinite(() -> {
            Shot shot = solve(robotPose.get(), robotVelocity.get(), target.get());
            targetVelocity = velocityFor(shot.distance) + velocityNudge;
        }).requiring(flywheel);
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
     * Keep the turret pointed at a field position (which may move), led for the robot's motion, plus the operator's
     * nudge. Runs until interrupted.
     */
    public Command aimAt(Supplier<Pose> robotPose, Supplier<Velocity> robotVelocity, Supplier<Pose> target) {
        return infinite(() -> {
            Shot shot = solve(robotPose.get(), robotVelocity.get(), target.get());
            lastShot = shot;
            setTurretTarget(shot.turretAngle + turretNudge, shot.turretRate);
        }).setEnd(end -> {
            // Hold the last angle; a leftover feedforward would drag the turret off it
            turretTargetRate = 0;
            lastShot = null;
        }).requiring(turret);
    }

    /** @param direction +1 left (CCW), -1 right */
    public Command nudgeTurret(int direction) {
        return instant(() -> turretNudge += direction * TURRET_NUDGE);
    }

    // ---- Aiming math ----

    /** Where to aim so a ball launched now reaches the target. Built by {@link #solve}. */
    public static class Shot {
        /** Field position to aim at: the target, moved back against the ball's inherited velocity. */
        public final Pose aimPoint;
        /** Inches from the turret pivot to the aim point. Use this for the flywheel speed. */
        public final double distance;
        /** Seconds from launch to the target, from TIME_OF_FLIGHT_TABLE. */
        public final double timeOfFlight;
        /** Robot-relative turret angle (radians) that points at the aim point. */
        public final double turretAngle;
        /** How fast turretAngle is changing (radians/sec) as the robot moves, for the turret feedforward. */
        public final double turretRate;
        /** Inches between the aim point and the real target. */
        public final double lead;

        Shot(Pose aimPoint, double distance, double timeOfFlight, double turretAngle, double turretRate, double lead) {
            this.aimPoint = aimPoint;
            this.distance = distance;
            this.timeOfFlight = timeOfFlight;
            this.turretAngle = turretAngle;
            this.turretRate = turretRate;
            this.lead = lead;
        }
    }

    /**
     * Aim at a target from a moving robot.
     * 1. Predict the pose AIM_LOOKAHEAD seconds ahead, so the turret isn't aiming from where the robot was.
     * 2. The ball leaves with the turret pivot's velocity (the robot's velocity, plus the spin swinging an off-center
     *    pivot around). Over its time of flight that carries it velocity × time past wherever it was aimed, so aim
     *    that far short of the target. The time of flight depends on the distance to the aim point, so iterate.
     * 3. Differentiate the angle to the aim point to get how fast the turret has to turn to stay on it.
     *
     * @param robotVelocity field-relative (inches/sec and radians/sec), as {@code Follower.velocity()} gives it
     */
    public static Shot solve(Pose robotPose, Velocity robotVelocity, Pose target) {
        double vx = robotVelocity.vx;
        double vy = robotVelocity.vy;
        double omega = robotVelocity.omega;

        Pose pose = new Pose(
                robotPose.x() + vx * AIM_LOOKAHEAD,
                robotPose.y() + vy * AIM_LOOKAHEAD,
                robotPose.heading() + omega * AIM_LOOKAHEAD);
        Pose pivot = turretPivot(pose);

        // Pivot velocity = robot velocity + omega × (pivot - robot center)
        double pivotVx = vx - omega * (pivot.y() - pose.y());
        double pivotVy = vy + omega * (pivot.x() - pose.x());

        double aimX = target.x();
        double aimY = target.y();
        double timeOfFlight = 0;
        if (SHOOT_ON_THE_MOVE) {
            // Converges in a few passes: time of flight changes slowly with distance
            for (int i = 0; i < 3; i++) {
                timeOfFlight = interpolate(TIME_OF_FLIGHT_TABLE, Math.hypot(aimX - pivot.x(), aimY - pivot.y()));
                aimX = target.x() - pivotVx * timeOfFlight;
                aimY = target.y() - pivotVy * timeOfFlight;
            }
        }

        double dx = aimX - pivot.x();
        double dy = aimY - pivot.y();
        double distanceSquared = Math.max(dx * dx + dy * dy, 1); // no blow-up when right on top of the target
        double fieldAngle = Math.atan2(dy, dx);
        // d/dt atan2(dy, dx) with the aim point still and the pivot moving; the robot's own turn takes the turret with it
        double fieldRate = (dy * pivotVx - dx * pivotVy) / distanceSquared;

        return new Shot(
                new Pose(aimX, aimY, 0),
                Math.sqrt(distanceSquared),
                timeOfFlight,
                Angle.normalizeSigned(fieldAngle - pose.heading()),
                fieldRate - omega,
                Math.hypot(aimX - target.x(), aimY - target.y()));
    }

    /** Inches from the turret pivot to a field position. */
    public double distanceTo(Pose robotPose, Pose target) {
        Pose pivot = turretPivot(robotPose);
        return Math.hypot(target.x() - pivot.x(), target.y() - pivot.y());
    }

    private static Pose turretPivot(Pose robotPose) {
        double cos = Math.cos(robotPose.heading());
        double sin = Math.sin(robotPose.heading());
        return new Pose(
                robotPose.x() + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin,
                robotPose.y() + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos,
                robotPose.heading());
    }

    static double velocityFor(double distance) {
        return interpolate(VELOCITY_TABLE, distance);
    }

    /** Linear interpolation over rows of {x, y} sorted by x, held at the end rows. */
    static double interpolate(double[][] table, double x) {
        if (x <= table[0][0]) return table[0][1];
        for (int i = 1; i < table.length; i++) {
            if (x <= table[i][0]) {
                double t = (x - table[i - 1][0]) / (table[i][0] - table[i - 1][0]);
                return table[i - 1][1] + t * (table[i][1] - table[i - 1][1]);
            }
        }
        return table[table.length - 1][1];
    }

    // ---- State ----

    public boolean isSpinning() {
        return targetVelocity != 0;
    }

    public boolean isFlywheelAtSpeed() {
        return isSpinning() && Math.abs(targetVelocity - flywheelVelocity) <= VELOCITY_TOLERANCE;
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
        telemetry.addData("Turret rate target / actual (deg/s)", "%.0f / %.0f",
                Math.toDegrees(turretTargetRate), Math.toDegrees(turretVelocity));
        telemetry.addData("Turret power", "%.2f", turret.getPower());
        if (lastShot != null) {
            telemetry.addData("Shot lead / flight", "%.1f in / %.2f s", lastShot.lead, lastShot.timeOfFlight);
        }
    }
}
