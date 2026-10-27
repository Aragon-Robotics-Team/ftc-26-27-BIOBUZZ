package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.TURRET_ENCODER_REVERSED;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.TURRET_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.TURRET_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.TURRET_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.TURRET_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.TURRET_MOTOR_TICKS_PER_REV;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the turret PID automatically with step tests: the turret moves STEP_DEG, holds, and the move is scored
 * (overshoot, ringing, shaking, how close it ends up). Each trial is one step each way, ±STEP_DEG/2 around where the
 * turret starts, through the same PID loop the Launcher runs.
 * Start with the turret facing forward: that's where it zeroes.
 *
 * 0. kS: ramps the power up slowly each way until the turret starts to move. That's the friction kS has to beat,
 *    so the PID doesn't stall a few degrees short of the target.
 * 1. P and D: raises P while the steps stay clean. When P overshoots, adds D (raising it) until the steps are clean
 *    again, then keeps raising P. Stops when D can't fix the overshoot or the turret starts to shake.
 * 2. I: only if the turret still stops short of the target. Raises I until it gets within the tolerance.
 *
 * The result prints as lines to paste into RobotConstants.Launcher. Check it in the Turret Tuner.
 */
@Configurable
@TeleOp(name = "Turret Auto Tuner", group = "Tuning")
public class TurretAutoTuner extends LinearOpMode {
    public static double STEP_DEG = 45;             // each step, across where the turret starts
    public static double STEP_WINDOW_S = 1.5;       // how long to watch each step
    public static double PAUSE_S = 0.5;             // hold before the next step
    public static double MAX_OVERSHOOT_DEG = 1;
    public static double TOLERANCE_DEG = 1;         // close enough to count as on target
    public static double P_START = 0.5, P_GROWTH = 1.4;
    public static double D_START = 0.01, D_GROWTH = 1.5, D_MAX = 0.3;
    public static double I_START = 2, I_GROWTH = 1.6, I_MAX = 60;
    public static double SHAKE_POWER = 0.1;         // power reversals bigger than this while holding count as shaking
    public static int MAX_TRIALS = 25;
    public static double SAFE_ANGLE_DEG = 60;       // cut power past this, or the soft limits if tighter
    public static double KS_RAMP = 0.1;             // power per second while finding kS
    public static double KS_MOVE_DEG = 1;           // this far counts as moving
    public static double KS_MAX = 0.4;              // give up finding kS past this power

    private Launcher launcher;
    private DcMotorEx turret;
    private TelemetryManager panels;
    private double safeAngle;
    private double ticksPerRadian;
    private double ks = 0;
    private final List<String> report = new ArrayList<>();
    private boolean aborted = false;

    /** How one set of gains handled a step each way (the worse of the two). */
    private static class TrialResult {
        double p, i, d;
        double overshootDeg;
        double settleSeconds = 0;   // last time outside the tolerance
        boolean settled = true;     // inside the tolerance for the end of the window
        double finalErrorDeg;       // mean |error| over the end of the window
        boolean ringing;            // swung back across the target more than once
        boolean shaking;            // power flipping back and forth while holding

        boolean clean() {
            return overshootDeg <= MAX_OVERSHOOT_DEG && !ringing && !shaking;
        }

        String describe() {
            return String.format("P %.3f I %.2f D %.4f: overshoot %.1f°, end error %.1f°, %s%s%s",
                    p, i, d, overshootDeg, finalErrorDeg,
                    settled ? String.format("settles %.2fs", settleSeconds) : "doesn't settle",
                    ringing ? ", rings" : "", shaking ? ", shakes" : "");
        }
    }

    @Override
    public void runOpMode() {
        double savedP = RobotConstants.Launcher.TURRET_KP;
        double savedI = RobotConstants.Launcher.TURRET_KI;
        double savedD = RobotConstants.Launcher.TURRET_KD;
        double savedS = RobotConstants.Launcher.TURRET_KS;
        Scheduler.reset();
        launcher = new Launcher(hardwareMap); // zeroes the turret where it is
        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
        safeAngle = Math.min(Math.toRadians(SAFE_ANGLE_DEG), Math.min(-TURRET_MIN_ANGLE, TURRET_MAX_ANGLE));
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // Before start the turret is unpowered: turning it CCW by hand should make the angle go up.
        // Read straight from the encoder, since launcher.update() would run the turret loop.
        ticksPerRadian =
                (TURRET_ENCODER_REVERSED ? -1 : 1) * TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO / (2 * Math.PI);
        while (opModeInInit()) {
            panels.addLine("Turret Auto Tuner");
            panels.addLine("Turret facing forward, nothing in its way. It will step " + Math.round(STEP_DEG / 2)
                    + "° each way, over and over.");
            panels.addLine("Press start. Press stop at any time to cut power.");
            panels.addData("Angle (deg)", Math.toDegrees(encoderAngle()));
            panels.update(telemetry);
        }
        waitForStart();
        setGains(0, 0, 0);
        launcher.update(); // pick up where the turret is now, without pushing it
        launcher.start();

        try {
            tune();
        } finally {
            RobotConstants.Launcher.TURRET_KP = savedP;
            RobotConstants.Launcher.TURRET_KI = savedI;
            RobotConstants.Launcher.TURRET_KD = savedD;
            RobotConstants.Launcher.TURRET_KS = savedS;
            Scheduler.reset();
        }
    }

    private void tune() {
        // ---- 0. kS ----
        // Twice each way, keeping the second: the first can just be the motor taking up the gear play
        double up = 0, down = 0;
        for (int direction : new int[]{1, 1, -1, -1}) {
            double power = breakaway(direction);
            if (aborted) return;
            if (Double.isNaN(power)) {
                report.add(String.format("The turret didn't move at %.2f power. Check for binding, or raise KS_MAX.", KS_MAX));
                finish();
                return;
            }
            if (direction > 0) up = power;
            else down = power;
        }
        setGains(0, 0, 0);
        launcher.update(); // catch up with where the ramps left the turret, without pushing it
        ks = (up + down) / 2;
        report.add(String.format("kS %.3f (starts moving at %.3f CCW, %.3f CW)", ks, up, down));

        double half = Math.toRadians(STEP_DEG / 2);
        // Get to one side first, so every scored step is the full STEP_DEG
        setGains(P_START, 0, 0);
        step(-half, "Moving to the start");
        if (aborted) return;

        // ---- 1. P and D ----
        double p = P_START, d = 0;
        TrialResult best = null;
        int trials = 0;
        while (trials++ < MAX_TRIALS) {
            TrialResult result = trial(p, 0, d);
            if (aborted) return;
            report.add(result.describe());
            if (result.clean()) {
                best = result;
                p *= P_GROWTH;
                continue;
            }
            if (result.shaking) break;
            // Overshoots or rings: more damping, same P
            d = d == 0 ? D_START : d * D_GROWTH;
            if (d > D_MAX) break;
        }
        if (best == null) {
            report.add("Even P_START overshoots. Lower P_START and run again.");
            finish();
            return;
        }

        // ---- 2. I, only if it stops short ----
        TrialResult chosen = best;
        if (!best.settled) {
            double i = I_START;
            while (trials++ < MAX_TRIALS && i <= I_MAX) {
                TrialResult result = trial(best.p, i, best.d);
                if (aborted) return;
                report.add(result.describe());
                if (!result.clean()) break;
                chosen = result;
                if (result.settled) break;
                i *= I_GROWTH;
            }
            if (!chosen.settled) report.add("Still stops short of the target: check for binding, or raise TOLERANCE_DEG.");
        }

        report.add("Paste into RobotConstants.Launcher:");
        report.add(String.format("public static double TURRET_KP = %.4f;", chosen.p));
        report.add(String.format("public static double TURRET_KI = %.4f;", chosen.i));
        report.add(String.format("public static double TURRET_KD = %.4f;", chosen.d));
        report.add(String.format("public static double TURRET_KS = %.4f;", ks));
        finish();
    }

    /** A step to +STEP_DEG/2 and back to -STEP_DEG/2 with these gains, scored as the worse of the two. */
    private TrialResult trial(double p, double i, double d) {
        setGains(p, i, d);
        double half = Math.toRadians(STEP_DEG / 2);
        String label = String.format("P %.3f I %.2f D %.4f", p, i, d);
        TrialResult there = step(half, label);
        if (aborted) return there;
        TrialResult back = step(-half, label);

        TrialResult result = new TrialResult();
        result.p = p;
        result.i = i;
        result.d = d;
        result.overshootDeg = Math.max(there.overshootDeg, back.overshootDeg);
        result.settled = there.settled && back.settled;
        result.settleSeconds = Math.max(there.settleSeconds, back.settleSeconds);
        result.finalErrorDeg = Math.max(there.finalErrorDeg, back.finalErrorDeg);
        result.ringing = there.ringing || back.ringing;
        result.shaking = there.shaking || back.shaking;
        return result;
    }

    /** Turn to the target, watch for STEP_WINDOW_S, then hold for PAUSE_S. */
    private TrialResult step(double target, String label) {
        TrialResult result = new TrialResult();
        double start = launcher.getTurretAngle();
        double direction = Math.signum(target - start);
        double tolerance = Math.toRadians(TOLERANCE_DEG);
        launcher.start(); // clears the integral from the last step
        schedule(launcher.turnTurretTo(target));

        double peak = 0, errorSum = 0, lastPower = 0;
        int lastSign = 0, crossings = 0, reversals = 0, errorSamples = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < STEP_WINDOW_S + PAUSE_S) {
            launcher.update();
            Scheduler.execute();
            double angle = launcher.getTurretAngle();
            if (Math.abs(angle) > safeAngle) {
                aborted = true;
                report.add(String.format("Stopped: the turret got to %.1f°, past the %.1f° safety limit. "
                                + "Check that + power turns it CCW and the angle goes up CCW.",
                        Math.toDegrees(angle), Math.toDegrees(safeAngle)));
                finish();
                return result;
            }
            double t = timer.seconds();
            double error = target - angle;
            if (t < STEP_WINDOW_S) {
                peak = Math.max(peak, (angle - target) * direction); // past the target, in the step's direction
                if (Math.abs(error) > tolerance) {
                    result.settleSeconds = t;
                    int sign = (int) Math.signum(error);
                    if (lastSign != 0 && sign != lastSign) crossings++;
                    lastSign = sign;
                }
                if (t > STEP_WINDOW_S - 0.25) {
                    errorSum += Math.abs(error);
                    errorSamples++;
                }
                // Shaking: the power keeps reversing once it should be holding
                if (t > STEP_WINDOW_S / 2) {
                    double power = turret.getPower();
                    if (Math.abs(power) > SHAKE_POWER && Math.abs(lastPower) > SHAKE_POWER
                            && Math.signum(power) != Math.signum(lastPower)) reversals++;
                    if (Math.abs(power) > SHAKE_POWER) lastPower = power;
                }
            }
            panels.addLine(label);
            panels.addData("Target (deg)", Math.toDegrees(target));
            panels.addData("Actual (deg)", Math.toDegrees(angle));
            panels.update(telemetry);
        }
        if (!opModeIsActive()) aborted = true;

        result.overshootDeg = Math.toDegrees(Math.max(0, peak));
        // Settled if it was inside the tolerance for the last third of the window
        result.settled = result.settleSeconds < STEP_WINDOW_S * 2 / 3;
        result.finalErrorDeg = errorSamples > 0 ? Math.toDegrees(errorSum / errorSamples) : 0;
        result.ringing = crossings >= 2;
        result.shaking = reversals >= 3;
        return result;
    }

    /**
     * Ramp the power up from 0 in one direction until the turret moves KS_MOVE_DEG, then let it stop.
     * @return the power it started moving at, or NaN if it never did
     */
    private double breakaway(int direction) {
        double start = encoderAngle();
        double power = 0;
        boolean moved = false;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && power <= KS_MAX) {
            power = KS_RAMP * timer.seconds();
            turret.setPower(direction * power);
            double angle = encoderAngle();
            if (Math.abs(angle - start) > Math.toRadians(KS_MOVE_DEG)) {
                moved = true;
                break;
            }
            panels.addLine("Finding kS: ramping the power " + (direction > 0 ? "CCW" : "CW"));
            panels.addData("Power", direction * power);
            panels.addData("Moved (deg)", Math.toDegrees(angle - start));
            panels.update(telemetry);
        }
        turret.setPower(0);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 0.3) idle();
        if (!opModeIsActive()) aborted = true;
        return moved ? power : Double.NaN;
    }

    private double encoderAngle() {
        return turret.getCurrentPosition() / ticksPerRadian;
    }

    private void setGains(double p, double i, double d) {
        RobotConstants.Launcher.TURRET_KP = p;
        RobotConstants.Launcher.TURRET_KI = i;
        RobotConstants.Launcher.TURRET_KD = d;
        RobotConstants.Launcher.TURRET_KS = ks;
    }

    /** Leave the turret off and keep the result on screen until stop. */
    private void finish() {
        Scheduler.reset();
        turret.setPower(0);
        while (opModeIsActive()) {
            for (String line : report) panels.addLine(line);
            panels.update(telemetry);
            sleep(100);
        }
    }
}
