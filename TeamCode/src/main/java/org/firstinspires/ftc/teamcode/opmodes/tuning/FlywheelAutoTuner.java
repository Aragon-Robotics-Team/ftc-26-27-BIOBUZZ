package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.FLYWHEEL_DIRECTION;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.FLYWHEEL_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.VELOCITY_TOLERANCE;

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
 * The hub's velocity reading is noisy (it's a difference of encoder counts), so every reading is averaged over
 * SMOOTH_S before it's judged, and the noise left after that is measured while holding the low speed. The settle band
 * and the overshoot limit both widen to NOISE_BANDS times that noise, so noise isn't mistaken for overshoot.
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
    public static double MAX_OVERSHOOT = 0.05;            // fraction of the step size, beyond the noise
    public static double SMOOTH_S = 0.1;                  // velocity readings are averaged over this long
    public static double NOISE_BANDS = 3;                 // noise allowance, in standard deviations of the smoothed speed
    // P values to try, as multiples of F (the FTC rule of thumb is 0.1)
    public static double[] P_MULTIPLIERS = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35};

    private DcMotorEx flywheel;
    private TelemetryManager panels;
    private final ElapsedTime displayTimer = new ElapsedTime();

    /** Velocity readings over time. */
    static class Trace {
        final List<Double> times = new ArrayList<>();
        final List<Double> velocities = new ArrayList<>();

        void add(double time, double velocity) {
            times.add(time);
            velocities.add(velocity);
        }

        int size() {
            return times.size();
        }
    }

    /** How one P value handled the step. */
    static class StepResult {
        double p;
        double settleSeconds = Double.NaN; // NaN = never settled inside the window
        double overshoot;                  // fraction of the step size, beyond the noise allowance
        int oscillations;                  // error sign flips outside the settle band
        double finalError;                 // mean error over the end of the window (ticks/sec)
        double rawNoise;                   // standard deviation of the readings while holding the low speed
        double noise;                      // the same, after smoothing
        double band;                       // settle band, ± ticks/sec

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
        report.add("Step tests at " + Math.round(testVelocity) + " ticks/sec (noise = ± one standard deviation):");
        for (double multiplier : P_MULTIPLIERS) {
            if (!opModeIsActive()) return;
            StepResult result = stepTest(multiplier * f, f, testVelocity);
            report.add(String.format("P %.2f: settle %s, overshoot %.0f%%, oscillations %d, noise ±%.0f raw / ±%.0f smoothed",
                    result.p, result.settled() ? String.format("%.2fs", result.settleSeconds) : "never",
                    result.overshoot * 100, result.oscillations, result.rawNoise, result.noise));

            boolean good = result.settled() && result.overshoot <= MAX_OVERSHOOT && result.oscillations <= 1;
            if (good && (best == null || result.settleSeconds < best.settleSeconds)) best = result;
            // More P only gets worse once it overshoots or rings
            if (result.overshoot > 3 * MAX_OVERSHOOT || result.oscillations >= 3) break;
        }
        flywheel.setPower(0);

        // ---- Result ----
        if (best == null) {
            report.add("No P settled without overshooting. If the noise is large, raise SMOOTH_S. Otherwise try "
                    + "smaller P_MULTIPLIERS, or tune P by hand in the Flywheel Tuner.");
            finish(report.toArray(new String[0]));
            return;
        }
        double i = 0;
        if (Math.abs(best.finalError) > best.band) {
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
        Trace trace = new Trace();
        double lastCheck = 0, lastVelocity = 0;
        while (opModeIsActive() && timer.seconds() < 8) {
            double t = timer.seconds();
            trace.add(t, flywheel.getVelocity());
            double velocity = smoothedAt(trace, trace.size() - 1);
            show("Measuring top speed", velocity, Double.NaN);
            // Level: changed less than 1% over the last half second, after at least 2 s
            if (t - lastCheck >= 0.5) {
                if (t > 2 && Math.abs(velocity - lastVelocity) < 0.01 * Math.abs(velocity)) break;
                lastVelocity = velocity;
                lastCheck = t;
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

        flywheel.setVelocity(low);
        Trace hold = record("P " + fmt(p) + ": settling low", low);
        flywheel.setVelocity(testVelocity);
        Trace step = record("P " + fmt(p) + ": step up", testVelocity);
        return analyze(p, hold, step, testVelocity);
    }

    /** Readings for STEP_WINDOW_S at the current target. */
    private Trace record(String phase, double target) {
        Trace trace = new Trace();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < STEP_WINDOW_S) {
            double velocity = flywheel.getVelocity();
            trace.add(timer.seconds(), velocity);
            show(phase, velocity, target);
        }
        return trace;
    }

    /**
     * Judge a step from the readings while holding the low speed and after stepping up to target.
     * The noise comes from the second half of the hold, when the speed has settled.
     */
    static StepResult analyze(double p, Trace hold, Trace step, double target) {
        StepResult result = new StepResult();
        result.p = p;

        // ---- Noise, from the second half of the hold ----
        double[] holdSmoothed = smooth(hold);
        double holdEnd = hold.size() > 0 ? hold.times.get(hold.size() - 1) : 0;
        List<Double> raw = new ArrayList<>(), smoothed = new ArrayList<>();
        for (int i = 0; i < hold.size(); i++) {
            if (hold.times.get(i) < holdEnd / 2) continue;
            raw.add(hold.velocities.get(i));
            smoothed.add(holdSmoothed[i]);
        }
        result.rawNoise = standardDeviation(raw);
        result.noise = standardDeviation(smoothed);
        double allowance = NOISE_BANDS * result.noise;
        result.band = Math.max(VELOCITY_TOLERANCE, allowance);

        // ---- Step response, smoothed ----
        double start = hold.size() > 0 ? holdSmoothed[hold.size() - 1] : 0;
        double stepSize = target - start;
        double[] velocities = smooth(step);
        double window = step.size() > 0 ? step.times.get(step.size() - 1) : 0;
        double lastOutsideBand = 0, peak = start, errorSum = 0;
        int lastSign = 0, errorSamples = 0;
        for (int i = 0; i < step.size(); i++) {
            double t = step.times.get(i);
            double error = target - velocities[i];
            peak = Math.max(peak, velocities[i]);
            if (Math.abs(error) > result.band) {
                lastOutsideBand = t;
                int sign = (int) Math.signum(error);
                if (lastSign != 0 && sign != lastSign) result.oscillations++;
                lastSign = sign;
            }
            if (t > window - 0.5) {
                errorSum += error;
                errorSamples++;
            }
        }
        // Settled if it was inside the band for the last quarter of the window
        if (lastOutsideBand < 0.75 * window) result.settleSeconds = lastOutsideBand;
        result.overshoot = stepSize > 0 ? Math.max(0, peak - target - allowance) / stepSize : 0;
        result.finalError = errorSamples > 0 ? errorSum / errorSamples : 0;
        return result;
    }

    /** Each reading averaged with the ones in the SMOOTH_S before it. */
    static double[] smooth(Trace trace) {
        double[] smoothed = new double[trace.size()];
        for (int i = 0; i < trace.size(); i++) smoothed[i] = smoothedAt(trace, i);
        return smoothed;
    }

    private static double smoothedAt(Trace trace, int index) {
        double end = trace.times.get(index);
        double sum = 0;
        int count = 0;
        for (int i = index; i >= 0 && trace.times.get(i) > end - SMOOTH_S; i--) {
            sum += trace.velocities.get(i);
            count++;
        }
        return sum / count;
    }

    private static double standardDeviation(List<Double> values) {
        if (values.size() < 2) return 0;
        double mean = 0;
        for (double value : values) mean += value / values.size();
        double squares = 0;
        for (double value : values) squares += (value - mean) * (value - mean);
        return Math.sqrt(squares / values.size());
    }

    /** At most 10 times a second, so sending telemetry doesn't slow the readings down. */
    private void show(String phase, double velocity, double target) {
        if (displayTimer.seconds() < 0.1) return;
        displayTimer.reset();
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
