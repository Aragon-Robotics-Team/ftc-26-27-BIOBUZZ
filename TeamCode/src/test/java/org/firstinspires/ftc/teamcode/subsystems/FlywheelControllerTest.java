package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.FlywheelControl;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class FlywheelControllerTest {
    private static final double TOP_SPEED_12V = 1850; // ticks/sec at full power on 12 V
    private static final double TIME_CONSTANT = 0.4;  // s, motor + flywheel
    private static final double TARGET = 1609;

    private FlywheelControl savedControl;
    private double savedKv, savedKs, savedKp, savedKi, savedKd;

    @Before
    public void save() {
        savedControl = RobotConstants.Launcher.FLYWHEEL_CONTROL;
        savedKv = RobotConstants.Launcher.FLYWHEEL_KV;
        savedKs = RobotConstants.Launcher.FLYWHEEL_KS;
        savedKp = RobotConstants.Launcher.FLYWHEEL_KP;
        savedKi = RobotConstants.Launcher.FLYWHEEL_KI;
        savedKd = RobotConstants.Launcher.FLYWHEEL_KD;
        RobotConstants.Launcher.FLYWHEEL_KV = 1 / TOP_SPEED_12V;
        RobotConstants.Launcher.FLYWHEEL_KS = 0;
        RobotConstants.Launcher.FLYWHEEL_KP = 2 / TOP_SPEED_12V;
        RobotConstants.Launcher.FLYWHEEL_KI = 0;
        RobotConstants.Launcher.FLYWHEEL_KD = 0;
    }

    @After
    public void restore() {
        RobotConstants.Launcher.FLYWHEEL_CONTROL = savedControl;
        RobotConstants.Launcher.FLYWHEEL_KV = savedKv;
        RobotConstants.Launcher.FLYWHEEL_KS = savedKs;
        RobotConstants.Launcher.FLYWHEEL_KP = savedKp;
        RobotConstants.Launcher.FLYWHEEL_KI = savedKi;
        RobotConstants.Launcher.FLYWHEEL_KD = savedKd;
    }

    /** First-order flywheel: speed heads for power × the top speed at this voltage. */
    private static double step(double velocity, double power, double volts, double dt) {
        double heading = power * TOP_SPEED_12V * volts / 12;
        return velocity + (heading - velocity) * (1 - Math.exp(-dt / TIME_CONSTANT));
    }

    /** Speed after `seconds` of control from `start`, with a loop every `dt`. */
    private static double run(FlywheelController controller, double start, double volts, double dt, double seconds) {
        double velocity = start;
        for (double t = 0; t < seconds; t += dt) {
            velocity = step(velocity, controller.power(TARGET, velocity, volts, dt), volts, dt);
        }
        return velocity;
    }

    @Test
    public void pidfHoldsTargetOnAFreshOrTiredBattery() {
        RobotConstants.Launcher.FLYWHEEL_CONTROL = FlywheelControl.PIDF;
        for (double volts : new double[]{14, 12.5}) {
            double velocity = run(new FlywheelController(), 0, volts, 0.02, 3);
            assertEquals("at " + volts + " V", TARGET, velocity, 5);
        }
    }

    @Test
    public void bangBangRecoversFasterThanPidf() {
        // A ball takes 15% of the speed: how much is back after 0.15 s, at a 20 ms loop?
        double dip = 0.85 * TARGET;
        RobotConstants.Launcher.FLYWHEEL_CONTROL = FlywheelControl.PIDF;
        double pidf = run(new FlywheelController(), dip, 12.5, 0.02, 0.15);
        RobotConstants.Launcher.FLYWHEEL_CONTROL = FlywheelControl.BANG_BANG;
        double bangBang = run(new FlywheelController(), dip, 12.5, 0.02, 0.15);
        assertTrue("PIDF " + pidf + ", bang-bang " + bangBang, bangBang > pidf);
        // And bang-bang still settles at the target, not above it
        assertEquals(TARGET, run(new FlywheelController(), 0, 12.5, 0.02, 3), 0.02 * TARGET);
    }

    @Test
    public void neverRunsBackwardsAndStopsAtZeroTarget() {
        RobotConstants.Launcher.FLYWHEEL_CONTROL = FlywheelControl.PIDF;
        FlywheelController controller = new FlywheelController();
        assertEquals(0, controller.power(TARGET, 3000, 12, 0.02), 0); // far above target: coast, don't brake
        assertEquals(0, controller.power(0, 1500, 12, 0.02), 0);
    }
}
