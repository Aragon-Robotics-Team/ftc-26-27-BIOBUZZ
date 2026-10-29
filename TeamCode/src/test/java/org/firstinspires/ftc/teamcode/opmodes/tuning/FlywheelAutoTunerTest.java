package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

public class FlywheelAutoTunerTest {
    private static final double TARGET = 1609;
    private static final double LOW = 0.8 * TARGET;
    private static final double STEP = TARGET - LOW;
    private static final double DT = 0.004;          // a reading every 4 ms
    private static final double RAW_NOISE = 25;      // ticks/sec, one standard deviation

    /** Holding the low speed, with noise. */
    private static FlywheelAutoTuner.Trace hold(Random random) {
        FlywheelAutoTuner.Trace trace = new FlywheelAutoTuner.Trace();
        for (double t = 0; t < 2; t += DT) trace.add(t, LOW + RAW_NOISE * random.nextGaussian());
        return trace;
    }

    /** A clean first-order rise to the target (no real overshoot), plus `hump` × step of real overshoot at 0.3 s. */
    private static FlywheelAutoTuner.Trace step(Random random, double hump) {
        FlywheelAutoTuner.Trace trace = new FlywheelAutoTuner.Trace();
        for (double t = 0; t < 2; t += DT) {
            double clean = TARGET - STEP * Math.exp(-t / 0.15) + hump * STEP * (t / 0.3) * Math.exp(1 - t / 0.3);
            trace.add(t, clean + RAW_NOISE * random.nextGaussian());
        }
        return trace;
    }

    @Test
    public void noiseAloneIsNotOvershoot() {
        Random random = new Random(1);
        FlywheelAutoTuner.StepResult result = FlywheelAutoTuner.analyze(0.79, hold(random), step(random, 0), TARGET);
        assertTrue("settled", result.settled());
        assertEquals(0, result.overshoot, 0.01);
        assertTrue("oscillations " + result.oscillations, result.oscillations <= 1);
        assertEquals(RAW_NOISE, result.rawNoise, 3);
        assertTrue("smoothing cuts the noise: " + result.noise, result.noise < RAW_NOISE / 3);
    }

    @Test
    public void realOvershootIsStillCaught() {
        Random random = new Random(2);
        FlywheelAutoTuner.StepResult result = FlywheelAutoTuner.analyze(0.79, hold(random), step(random, 0.2), TARGET);
        assertTrue("overshoot " + result.overshoot, result.overshoot > FlywheelAutoTuner.MAX_OVERSHOOT);
    }
}
