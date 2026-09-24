package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.*;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Turreted flywheel shooter. The flywheel is velocity-controlled; the turret runs a PID loop on its encoder angle. */
public class Launcher implements Subsystem {
    public enum State {
        IDLE,
        SPINNING_UP,
        READY
    }

    private final DcMotorEx flywheel;
    private final DcMotorEx turret;
    private final double turretTicksPerRadian = TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO / (2 * Math.PI);

    private State state = State.IDLE;
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

    // ---- Flywheel ----

    public void spinUp() {
        spinUp(TARGET_VELOCITY);
    }

    /** @param velocity ticks/sec */
    public void spinUp(double velocity) {
        // TODO: velocity from distance
        targetVelocity = velocity;
        state = State.SPINNING_UP;
    }

    public void idle() {
        targetVelocity = 0;
        state = State.IDLE;
    }

    public State getState() {
        return state;
    }

    /** Push new velocity PIDF to the motor controller. Used by the tuning OpMode. */
    public void setFlywheelPIDF(PIDFCoefficients pidf) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    // ---- Turret ----

    /**
     * Point the turret at a robot-relative angle (radians, 0 = forward, CCW positive).
     * Clamped to [TURRET_MIN_ANGLE, TURRET_MAX_ANGLE], so targets past the limits need the drivetrain to turn.
     */
    public void setTurretAngle(double angle) {
        turretTargetAngle = Range.clip(Angle.normalizeSigned(angle), TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
    }

    /** Point the turret at a field position, given where the robot is. */
    public void aimAt(Pose robotPose, Pose target) {
        // TODO: turret offset from center
        double fieldAngle = Math.atan2(target.y() - robotPose.y(), target.x() - robotPose.x());
        setTurretAngle(fieldAngle - robotPose.heading());
    }

    /** Tell the launcher where the turret is right now, e.g. the angle Auto left it at. */
    public void setCurrentTurretAngle(double angle) {
        turretAngleOffset = angle - turret.getCurrentPosition() / turretTicksPerRadian;
        turretAngle = angle;
        lastTurretAngle = angle;
    }

    /** Robot-relative turret angle in radians, as of the last {@link #update()}. */
    public double getTurretAngle() {
        return turretAngle;
    }

    public boolean isTurretOnTarget() {
        return Math.abs(turretTargetAngle - getTurretAngle()) <= TURRET_ANGLE_TOLERANCE;
    }

    // ---- Shooting ----

    /** Flywheel is at speed and the turret is on target. */
    public boolean isReady() {
        return state == State.READY && isTurretOnTarget();
    }

    /** Fire a game piece. Only meaningful once {@link #isReady()}. */
    public void launch() {
        // TODO: fire feeder / gate
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
        if (state == State.IDLE) {
            flywheel.setPower(0);
        } else {
            flywheel.setVelocity(targetVelocity);
            // Drops back to SPINNING_UP when a shot pulls the speed down
            state = Math.abs(targetVelocity - flywheelVelocity) <= VELOCITY_TOLERANCE ? State.READY : State.SPINNING_UP;
        }

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
        idle();
        flywheel.setPower(0);
        turret.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Launcher", state);
        telemetry.addData("Flywheel target / actual", "%.0f / %.0f", targetVelocity, flywheelVelocity);
        telemetry.addData("Turret target / actual (deg)", "%.1f / %.1f",
                Math.toDegrees(turretTargetAngle), Math.toDegrees(getTurretAngle()));
    }
}
