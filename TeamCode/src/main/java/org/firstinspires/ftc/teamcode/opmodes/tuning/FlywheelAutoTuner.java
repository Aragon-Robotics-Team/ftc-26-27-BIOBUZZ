package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.FLYWHEEL_DIRECTION;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.FLYWHEEL_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.VELOCITY_TOLERANCE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the flywheel velocity PIDF automatically. Run with a charged battery and nothing touching the flywheel.
 *
 * 1. Full power until the speed levels off: F = 32767 / top speed (the hub's feedforward scale).
 * 2. With that F, steps the target from 80% up to the test speed (like recovering after a shot) for a range of
 *    P values, and keeps the P that settles fastest without overshooting more than MAX_OVERSHOOT.
 * I and D stay 0 unless it can't reach the target, in which case it suggests a small I.
 *
 * The result prints as a line to paste into RobotConstants.Launcher. Fine-tune it in the Flywheel Tuner,
 * and check recovery with real shots.
 */
@Configurable
@TeleOp(name = "Flywheel Auto Tuner", group = "Tuning")
public class FlywheelAutoTuner extends LinearOpMode {
    public static double TEST_VELOCITY = TARGET_VELOCITY; // ticks/sec, capped at 85% of the measured top speed
    public static double STEP_FRACTION = 0.8;             // steps from this fraction of the test speed up to it
    public static double STEP_WINDOW_S = 2.0;             // how long to watch each step
    public static double MAX_OVERSHOOT = 0.05;            // fraction of the step size
    // P values to try, as multiples of F (the FTC rule of thumb is 0.1)
    public static double[] P_MULTIPLIERS = {0.05, 0.1, 0.2, 0.4, 0.8, 1.6, 3.2};

    private DcMotorEx flywheel;
    private TelemetryManager panels;

    /** How one P value handled the step. */
    private static class StepResult {
        double p;
        double settleSeconds = Double.NaN; // NaN = never settled inside the window
        double overshoot;                  // fraction of the step size
        int oscillations;                  // error sign flips outside the tolerance band
        double finalError;                 // mean error over the end of the window (ticks/sec)

        boolean settled() {
            return !Double.isNaN(settleSeconds);
        }
    }

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheel.setDirection(FLYWHEEL_DIRECTION);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        panels.addLine("Flywheel Auto Tuner");
        panels.addLine("Charged battery, nothing touching the flywheel. The flywheel will run at full power.");
        panels.addLine("Press start. Press stop at any time to cut power.");
        panels.update(telemetry);
        waitForStart();

        // ---- 1. Top speed -> F ----
        double voltage = batteryVoltage();
        double maxVelocity = measureMaxVelocity();
        if (!opModeIsActive()) return;
        if (maxVelocity < -100) {
            // The hub's velocity loop would push the wrong way and run away, so don't go on
            finish("The encoder counts backwards: + power gave " + Math.round(maxVelocity) + " ticks/sec. "
                    + "Swap the encoder, or the motor wires and FLYWHEEL_DIRECTION, so + power reads + velocity.");
            return;
        }
        if (maxVelocity < 100) {
            finish("The flywheel barely moved (" + Math.round(maxVelocity) + " ticks/sec). Check the motor name, "
                    + "wiring and encoder cable, and that FLYWHEEL_DIRECTION makes + power spin it.");
            return;
        }
        double f = 32767 / maxVelocity;
        double testVelocity = Math.min(TEST_VELOCITY, 0.85 * maxVelocity);

        // ---- 2. Step tests over P ----
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        StepResult best = null;
        List<String> report = new ArrayList<>();
        report.add(String.format("Top speed %.0f ticks/sec at %.1f V  ->  F = %.2f", maxVelocity, voltage, f));
        report.add("Step tests at " + Math.round(testVelocity) + " ticks/sec:");
        for (double multiplier : P_MULTIPLIERS) {
            if (!opModeIsActive()) return;
            StepResult result = stepTest(multiplier * f, f, testVelocity);
            report.add(String.format("P %.2f: settle %s, overshoot %.0f%%, oscillations %d",
                    result.p, result.settled() ? String.format("%.2fs", result.settleSeconds) : "never",
                    result.overshoot * 100, result.oscillations));

            boolean good = result.settled() && result.overshoot <= MAX_OVERSHOOT && result.oscillations <= 1;
            if (good && (best == null || result.settleSeconds < best.settleSeconds)) best = result;
            // More P only gets worse once it overshoots or rings
            if (result.overshoot > 3 * MAX_OVERSHOOT || result.oscillations >= 3) break;
        }
        flywheel.setPower(0);

        // ---- Result ----
        if (best == null) {
            report.add("No P settled without overshooting. Try smaller P_MULTIPLIERS, or tune P by hand in the Flywheel Tuner.");
            finish(report.toArray(new String[0]));
            return;
        }
        double i = 0;
        if (Math.abs(best.finalError) > VELOCITY_TOLERANCE) {
            i = 0.1 * best.p;
            report.add(String.format("Settles %.0f ticks/sec off target, so I is set to 0.1 x P.", best.finalError));
        }
        report.add(String.format("Best: P %.2f (settles in %.2fs). Paste into RobotConstants.Launcher:", best.p, best.settleSeconds));
        report.add(String.format("public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(%.3f, %.3f, 0, %.3f);", best.p, i, f));
        finish(report.toArray(new String[0]));
    }

    /** Full power until the speed stops rising, then the average over the last half second (signed). */
    private double measureMaxVelocity() {
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(1.0);
        ElapsedTime timer = new ElapsedTime();
        double lastCheck = 0, lastVelocity = 0, velocity = 0;
        while (opModeIsActive() && timer.seconds() < 8) {
            velocity = flywheel.getVelocity();
            show("Measuring top speed", velocity, Double.NaN);
            // Level: changed less than 1% over the last half second, after at least 2 s
            if (timer.seconds() - lastCheck >= 0.5) {
                if (timer.seconds() > 2 && Math.abs(velocity - lastVelocity) < 0.01 * Math.abs(velocity)) break;
                lastVelocity = velocity;
                lastCheck = timer.seconds();
            }
        }
        double sum = 0;
        int samples = 0;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 0.5) {
            sum += flywheel.getVelocity();
            samples++;
        }
        flywheel.setPower(0);
        return samples > 0 ? sum / samples : 0;
    }

    /** Settle at the low speed, then step up to the test speed and watch the response. */
    private StepResult stepTest(double p, double f, double testVelocity) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0, f));
        double low = STEP_FRACTION * testVelocity;
        StepResult result = new StepResult();
        result.p = p;

        flywheel.setVelocity(low);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < STEP_WINDOW_S) show("P " + fmt(p) + ": settling low", flywheel.getVelocity(), low);

        double start = flywheel.getVelocity();
        double step = testVelocity - start;
        flywheel.setVelocity(testVelocity);
        timer.reset();
        double lastOutsideBand = 0, peak = start, errorSum = 0;
        int lastSign = 0, errorSamples = 0;
        while (opModeIsActive() && timer.seconds() < STEP_WINDOW_S) {
            double velocity = flywheel.getVelocity();
            double error = testVelocity - velocity;
            double t = timer.seconds();
            peak = Math.max(peak, velocity);
            if (Math.abs(error) > VELOCITY_TOLERANCE) {
                lastOutsideBand = t;
                int sign = (int) Math.signum(error);
                if (lastSign != 0 && sign != lastSign) result.oscillations++;
                lastSign = sign;
            }
            if (t > STEP_WINDOW_S - 0.5) {
                errorSum += error;
                errorSamples++;
            }
            show("P " + fmt(p) + ": step up", velocity, testVelocity);
        }
        // Settled if it was inside the band for the last quarter of the window
        if (lastOutsideBand < 0.75 * STEP_WINDOW_S) result.settleSeconds = lastOutsideBand;
        result.overshoot = step > 0 ? Math.max(0, peak - testVelocity) / step : 0;
        result.finalError = errorSamples > 0 ? errorSum / errorSamples : 0;
        return result;
    }

    private void show(String phase, double velocity, double target) {
        panels.addLine(phase);
        panels.addData("Actual", velocity);
        if (!Double.isNaN(target)) panels.addData("Target", target);
        panels.update(telemetry);
    }

    /** Leave the flywheel off and keep the result on screen until stop. */
    private void finish(String... lines) {
        flywheel.setPower(0);
        while (opModeIsActive()) {
            for (String line : lines) panels.addLine(line);
            panels.update(telemetry);
            sleep(100);
        }
    }

    private double batteryVoltage() {
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) voltage = Math.min(voltage, v);
        }
        return voltage;
    }

    private static String fmt(double value) {
        return String.format("%.2f", value);
    }
}
