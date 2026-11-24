package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.ShotSolver;
import org.firstinspires.ftc.teamcode.core.ShotSolver.Shot;
import org.firstinspires.ftc.teamcode.core.ShotTable;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Turreted flywheel shooter. The flywheel holds its speed with our own loop ({@link FlywheelController}, PIDF or
 * bang-bang, see FLYWHEEL_CONTROL), not the hub's; the turret runs a PID loop on its encoder angle,
 * plus a velocity feedforward so it can track a target while the robot moves.
 * Aiming simulates every shot ({@link ShotSolver}): the turret heading comes from a flight model with drag that
 * allows for the robot's motion. The flywheel speed comes from the measured {@link ShotTable} by distance (when it
 * has entries and USE_SHOT_TABLE is on), else from the same model. "Ready" means both are within the error that shot
 * can take. The flywheel allows for at most FLYWHEEL_MOVE_LIMIT of the robot's speed: it can't brake, so it shouldn't
 * spin up for a fast drive that ends in a standing shot.
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
    private final VoltageSensor battery;
    private final double turretTicksPerRadian =
            (TURRET_ENCODER_REVERSED ? -1 : 1) * TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO / (2 * Math.PI);

    private double targetVelocity = 0;
    private double flywheelVelocity = 0;
    private double flywheelPower = 0;
    private final FlywheelController flywheelController = new FlywheelController();
    private final ElapsedTime flywheelTimer = new ElapsedTime();
    // Battery voltage for the flywheel feedforward. Read a few times a second: it's a separate hub read, not bulk
    private double batteryVolts = 12;
    private final ElapsedTime batteryTimer = new ElapsedTime();
    private double turretTargetAngle = 0;
    private double turretTargetRate = 0; // rad/s the target is moving at, for the feedforward
    private boolean turretTargetClipped = false; // the wanted angle is past a soft limit
    private double turretWantedAngle = 0; // where the shot wants the turret, before clipping to the soft limits
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
    // The flywheel's own solver while the robot is faster than FLYWHEEL_MOVE_LIMIT (a different shot from the turret's)
    private final ShotSolver limitedSolver = new ShotSolver();
    // The shots the flywheel and turret are following, while enableFlywheel() / aimAt() run
    private Shot flywheelShot = null;
    private Shot turretShot = null;
    // While enableFlywheel() runs with the shot table: the distance it looked up and the speed it gave (before nudge)
    private boolean usingTable = false;
    private double tableDistance = 0;
    private double tableVelocity = 0;
    // The shot needs more than FLYWHEEL_SPEED_CAP: there's no shot, and the flywheel holds lastInRange instead
    private boolean overCap = false;
    // The last flywheel speed (before the nudge) for a shot within the cap, NaN before the first
    private double lastInRange = Double.NaN;

    public Launcher(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheel.setDirection(FLYWHEEL_DIRECTION);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, don't brake the flywheel
        // FlywheelController sets the power; the encoder still counts, so getVelocity() works
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        battery = hardwareMap.voltageSensor.iterator().next();

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
            Pose pose = robotPose.get();
            Velocity actual = robotVelocity.get();
            Velocity velocity = limitForFlywheel(actual);
            Pose goal = target.get();
            // Under the limit it's the turret's shot, and the solver solves it once for both
            flywheelShot = (velocity == actual ? shotSolver : limitedSolver).solve(pose, velocity, goal);
            usingTable = USE_SHOT_TABLE && !ShotTable.isEmpty();
            if (usingTable) {
                tableDistance = tableDistance(pose, velocity, goal, flywheelShot);
                tableVelocity = ShotTable.velocityAt(tableDistance);
                targetVelocity = tableVelocity + velocityNudge;
            } else {
                targetVelocity = flywheelShot.flywheelVelocity + velocityNudge;
            }
            // Too far to score: hold a shooting speed rather than spin up for it, so stopping to shoot doesn't wait
            // for the flywheel to coast back down (it can't brake)
            overCap = targetVelocity > FLYWHEEL_SPEED_CAP;
            if (overCap) {
                double hold = Double.isNaN(lastInRange) ? FLYWHEEL_OUT_OF_RANGE_VELOCITY : lastInRange;
                targetVelocity = Math.min(hold + velocityNudge, FLYWHEEL_SPEED_CAP);
            } else {
                lastInRange = targetVelocity - velocityNudge;
            }
        }).setEnd(end -> {
            flywheelShot = null;
            usingTable = false;
            overCap = false;
        }).requiring(flywheel);
    }

    /**
     * Hold whatever speed the supplier gives (ticks/sec), no nudge. For the Shot Table Calibration, where the
     * operator sets the speed by hand. Runs until {@link #idle()}.
     */
    public Command holdVelocity(DoubleSupplier velocity) {
        return infinite(() -> targetVelocity = velocity.getAsDouble()).requiring(flywheel);
    }

    /** The robot's velocity slowed to FLYWHEEL_MOVE_LIMIT (the same object if it's already under). */
    private static Velocity limitForFlywheel(Velocity velocity) {
        double speed = Math.hypot(velocity.vx, velocity.vy);
        double limit = Math.max(0, FLYWHEEL_MOVE_LIMIT);
        if (speed <= limit) return velocity;
        double scale = limit / speed;
        return new Velocity(velocity.vx * scale, velocity.vy * scale, velocity.omega);
    }

    /**
     * The distance to look up in the shot table. The table is for standing shots, so a moving robot looks up the
     * distance from where the ball's inherited motion would carry the turret by the time the ball arrives (the
     * "virtual target"): driving toward the hive needs a shorter shot, away a longer one. Taken from the same
     * look-ahead pose the solver aims from. Standing still, it's just {@link #distanceTo}.
     */
    private double tableDistance(Pose pose, Velocity velocity, Pose target, Shot shot) {
        Pose ahead = new Pose(
                pose.x() + velocity.vx * AIM_LOOKAHEAD,
                pose.y() + velocity.vy * AIM_LOOKAHEAD,
                pose.heading() + velocity.omega * AIM_LOOKAHEAD);
        Pose pivot = ShotSolver.turretPivot(ahead);
        double x = pivot.x(), y = pivot.y();
        if (SHOOT_ON_THE_MOVE && shot.feasible) {
            x += velocity.vx * shot.timeOfFlight;
            y += velocity.vy * shot.timeOfFlight;
        }
        return Math.hypot(target.x() - x, target.y() - y);
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
        if (usingTable) {
            // The model's margin as a fraction of the speed, applied to the table's speed
            if (flywheelShot == null || !flywheelShot.feasible || flywheelShot.exitSpeed <= 0) return SHOT_TABLE_TOLERANCE;
            return tableVelocity * flywheelShot.speedTolerance / flywheelShot.exitSpeed * READY_WINDOW_FRACTION;
        }
        if (flywheelShot == null || !flywheelShot.feasible) return VELOCITY_TOLERANCE;
        return flywheelShot.speedTolerance * FLYWHEEL_TICKS_PER_EXIT_SPEED * READY_WINDOW_FRACTION;
    }

    /** True while the flywheel speed comes from the shot table rather than the shot model. */
    public boolean isUsingShotTable() {
        return usingTable;
    }

    /** The distance the shot table was last looked up at (inches), while {@link #isUsingShotTable()}. */
    public double getTableDistance() {
        return tableDistance;
    }

    public double getFlywheelVelocity() {
        return flywheelVelocity;
    }

    /** Power the flywheel control set this loop, 0 when off. */
    public double getFlywheelPower() {
        return flywheelPower;
    }

    /** Battery volts the flywheel feedforward used this loop (read a few times a second while spinning). */
    public double getBatteryVolts() {
        return batteryVolts;
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

    /** The turret's angle read from its encoder now, radians: for init, when update() (which drives it) doesn't run. */
    public double readTurretAngle() {
        return turret.getCurrentPosition() / turretTicksPerRadian + turretAngleOffset;
    }

    public double getTurretTargetAngle() {
        return turretTargetAngle;
    }

    public double getTurretNudge() {
        return turretNudge;
    }

    /**
     * Within the angle error the current shot can take (TURRET_ANGLE_TOLERANCE when not aiming at a target) of where
     * the shot wants the turret, before clipping. A target just past a soft limit, with the turret parked on it, still
     * counts when it's that close; one deep in the blind zone behind the robot is far off and doesn't.
     */
    public boolean isTurretOnTarget() {
        return Math.abs(Angle.normalizeSigned(turretWantedAngle - getTurretAngle())) <= turretTolerance();
    }

    /**
     * The turret is held at a soft limit (TURRET_MIN_ANGLE / TURRET_MAX_ANGLE), usually because the shot needs it
     * turned past one, so the drivetrain has to turn instead.
     */
    public boolean isTurretAtLimit() {
        return turretTargetAngle == TURRET_MIN_ANGLE || turretTargetAngle == TURRET_MAX_ANGLE;
    }

    private double turretTolerance() {
        if (turretShot == null || !turretShot.feasible) return TURRET_ANGLE_TOLERANCE;
        return turretShot.angleTolerance * READY_WINDOW_FRACTION;
    }

    /** False when aiming at a target that no flywheel speed can reach from here (e.g. too close for the hood). */
    public boolean hasShot() {
        if (overCap) return false; // farther than we ever score from
        // With the table, "in range" is the table's measured range: the model's reach no longer decides it.
        // An infeasible model shot still points the turret straight at the target, which is right standing still.
        if (usingTable) return ShotTable.covers(tableDistance);
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

    /** Tell the launcher where the turret is right now, e.g. the angle Auto left it at. */
    public void setCurrentTurretAngle(double angle) {
        turretAngleOffset = angle - turret.getCurrentPosition() / turretTicksPerRadian;
        turretAngle = angle;
        lastTurretAngle = angle;
    }

    /**
     * Kept to [TURRET_MIN_ANGLE, TURRET_MAX_ANGLE], so targets past the limits need the drivetrain to turn. Past
     * them, the turret parks on the limit the target left through and stays there until the target is back in range
     * (TurretLimits), rather than swinging between the limits as the target crosses straight behind the robot.
     * @param rate how fast the target is moving (radians/sec), for the feedforward
     */
    private void setTurretTarget(double angle, double rate) {
        turretWantedAngle = Angle.normalizeSigned(angle);
        turretTargetAngle = TurretLimits.target(turretWantedAngle, turretTargetAngle, turretTargetClipped,
                TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
        turretTargetClipped = turretTargetAngle != turretWantedAngle;
        // Parked on a soft limit: don't push into it
        turretTargetRate = turretTargetClipped ? 0 : rate;
    }

    @Override
    public void start() {
        flywheelTimer.reset();
        flywheelController.reset();
        turretTimer.reset();
        turretIntegral = 0;
        lastTurretError = 0;
        lastTurretAngle = turretAngle;
    }

    @Override
    public void update() {
        flywheelVelocity = flywheel.getVelocity();
        turretAngle = turret.getCurrentPosition() / turretTicksPerRadian + turretAngleOffset;
        updateFlywheel();
        updateTurret();
    }

    private void updateFlywheel() {
        double dt = flywheelTimer.seconds();
        flywheelTimer.reset();
        if (isSpinning() && batteryTimer.seconds() > 0.25) {
            batteryTimer.reset();
            double volts = battery.getVoltage();
            if (volts > 6) batteryVolts = volts; // ignore a bad read
        }
        flywheelPower = flywheelController.power(targetVelocity, flywheelVelocity, batteryVolts, dt);
        flywheel.setPower(flywheelPower);
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
        if (Math.abs(error) > TURRET_KS_DEADBAND) power += TURRET_KS * Math.signum(error);
        turret.setPower(Range.clip(power, -TURRET_MAX_POWER, TURRET_MAX_POWER));
    }

    @Override
    public void stop() {
        targetVelocity = 0;
        flywheelPower = 0;
        flywheel.setPower(0);
        turret.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Launcher ready", isReady());
        telemetry.addData("Flywheel target / actual", "%.0f / %.0f", targetVelocity, flywheelVelocity);
        if (overCap) telemetry.addData("Flywheel", "too far to score (needs over %.0f): holding %.0f",
                FLYWHEEL_SPEED_CAP, targetVelocity);
        telemetry.addData("Flywheel power (" + FLYWHEEL_CONTROL + ")", "%.2f at %.1f V", flywheelPower, batteryVolts);
        telemetry.addData("Turret target / actual (deg)", "%.1f / %.1f",
                Math.toDegrees(turretTargetAngle), Math.toDegrees(getTurretAngle()));
        if (turretTargetClipped) {
            telemetry.addData("Turret wanted (deg)", "%.1f, past its limit", Math.toDegrees(turretWantedAngle));
        }
        telemetry.addData("Turret turn speed target / actual (deg/s)", "%.0f / %.0f",
                Math.toDegrees(turretTargetRate), Math.toDegrees(turretVelocity));
        telemetry.addData("Turret power", "%.2f", turret.getPower());
        if (usingTable) {
            telemetry.addData("Shot table", "%.0f in → %.0f ticks/s%s   [%s]", tableDistance, tableVelocity,
                    ShotTable.covers(tableDistance) ? "" : " (out of range)", ShotTable.getSource());
        } else if (USE_SHOT_TABLE) {
            telemetry.addLine("Shot table empty: flywheel speed from the shot model");
        }
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
