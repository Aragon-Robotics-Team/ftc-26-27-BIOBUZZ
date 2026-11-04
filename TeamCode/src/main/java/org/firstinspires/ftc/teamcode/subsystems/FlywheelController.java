package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.*;

import org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.FlywheelControl;

/**
 * Flywheel speed control in our own loop, instead of the hub's RUN_USING_ENCODER velocity loop: the motor runs
 * RUN_WITHOUT_ENCODER and gets the power this works out, once per loop.
 * - PIDF: battery-compensated feedforward (FLYWHEEL_KS + FLYWHEEL_KV × target) plus PID on the speed error.
 * - BANG_BANG: full power below target, the feedforward at or above it.
 * The gains are read from RobotConstants.Launcher on every call, so the tuners can change them live. No hardware, so
 * it can be unit tested.
 */
public class FlywheelController {
    private double integral = 0;
    private double lastError = Double.NaN;

    /** Forget the integral and the last error, e.g. when the flywheel stops. */
    public void reset() {
        integral = 0;
        lastError = Double.NaN;
    }

    /**
     * @param target   ticks/sec; 0 turns the flywheel off
     * @param velocity measured ticks/sec
     * @param volts    battery voltage
     * @param dt       seconds since the last call
     * @return motor power from 0 to 1. Never reverse: above target, the flywheel coasts down
     */
    public double power(double target, double velocity, double volts, double dt) {
        if (target <= 0) {
            reset();
            return 0;
        }
        double feedforward = (FLYWHEEL_KS + FLYWHEEL_KV * target) * 12 / volts;
        double error = target - velocity;
        if (FLYWHEEL_CONTROL == FlywheelControl.BANG_BANG) {
            reset();
            return error > 0 ? 1 : clip(feedforward);
        }

        if (dt > 0) {
            // Anti-windup: dump the integral when the error crosses zero, like the turret
            if (!Double.isNaN(lastError) && Math.signum(error) != Math.signum(lastError)) integral = 0;
            integral += error * dt;
        }
        double derivative = Double.isNaN(lastError) || dt <= 0 ? 0 : (error - lastError) / dt;
        lastError = error;
        return clip(feedforward + FLYWHEEL_KP * error + FLYWHEEL_KI * integral + FLYWHEEL_KD * derivative);
    }

    private static double clip(double power) {
        return Math.max(0, Math.min(1, power));
    }
}
