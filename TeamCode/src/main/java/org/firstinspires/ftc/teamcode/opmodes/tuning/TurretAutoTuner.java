package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.TURRET_DIRECTION;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.TURRET_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.TURRET_MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.TURRET_MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.TURRET_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.TURRET_MOTOR_TICKS_PER_REV;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the turret PID automatically with a relay test (the safe way to get the Ziegler-Nichols numbers).
 * Start with the turret facing forward: that's where it zeroes, and it oscillates around there.
 *
 * The turret is driven at +RELAY_POWER / -RELAY_POWER, flipping each time it crosses forward, which makes a small
 * steady oscillation. Its size and period give the ultimate gain Ku and period Tu, and from those:
 * - "no overshoot" gains (the default to paste, gentle, good for aiming)
 * - classic Ziegler-Nichols gains (faster, about 25% overshoot)
 *
 * If the turret doesn't move, raise RELAY_POWER; if it swings too far, lower it.
 * Fine-tune the result in the Turret Tuner.
 */
@Configurable
@TeleOp(name = "Turret Auto Tuner", group = "Tuning")
public class TurretAutoTuner extends LinearOpMode {
    public static double RELAY_POWER = 0.25;
    public static double HYSTERESIS_DEG = 0.5; // ignore crossings smaller than this (encoder noise, backlash)
    public static int WARMUP_CYCLES = 3;       // cycles to skip while the oscillation settles
    public static int MEASURE_CYCLES = 5;
    public static double SAFE_ANGLE_DEG = 30;  // cut power past this, or the soft limits if tighter
    public static double TIMEOUT_S = 20;

    private DcMotorEx turret;
    private TelemetryManager panels;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR_NAME);
        turret.setDirection(TURRET_DIRECTION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double ticksPerRadian = TURRET_MOTOR_TICKS_PER_REV * TURRET_GEAR_RATIO / (2 * Math.PI);
        double safeAngle = Math.min(Math.toRadians(SAFE_ANGLE_DEG), Math.min(-TURRET_MIN_ANGLE, TURRET_MAX_ANGLE));
        double hysteresis = Math.toRadians(HYSTERESIS_DEG);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        panels.addLine("Turret Auto Tuner");
        panels.addLine("Turret facing forward, nothing in its way. It will swing back and forth a little.");
        panels.addLine("Press start. Press stop at any time to cut power.");
        panels.update(telemetry);
        waitForStart();

        // Relay around forward (angle 0). Each crossing upward through 0 starts a new cycle.
        List<Double> crossingTimes = new ArrayList<>();
        List<Double> amplitudes = new ArrayList<>(); // half peak-to-peak of each cycle
        double power = RELAY_POWER; // kick it one way to get going
        double cycleMax = Double.NEGATIVE_INFINITY, cycleMin = Double.POSITIVE_INFINITY;
        double lastAngle = 0;
        int cyclesNeeded = WARMUP_CYCLES + MEASURE_CYCLES;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && crossingTimes.size() <= cyclesNeeded) {
            double angle = turret.getCurrentPosition() / ticksPerRadian;
            double t = timer.seconds();

            if (Math.abs(angle) > safeAngle) {
                finish(String.format("Stopped: the turret swung to %.1f°, past the %.1f° safety limit. Lower RELAY_POWER.",
                        Math.toDegrees(angle), Math.toDegrees(safeAngle)));
                return;
            }
            if (t > TIMEOUT_S || (crossingTimes.isEmpty() && t > 3)) {
                finish(crossingTimes.isEmpty()
                        ? "Stopped: the turret never swung back through forward. Raise RELAY_POWER, and check TURRET_DIRECTION (+ power should turn it CCW)."
                        : "Stopped: timed out before enough cycles. Check for binding, or raise RELAY_POWER.");
                return;
            }

            // Relay with hysteresis: push toward forward, flip only once clearly past it
            if (angle < -hysteresis) power = RELAY_POWER;
            else if (angle > hysteresis) power = -RELAY_POWER;
            turret.setPower(power);

            cycleMax = Math.max(cycleMax, angle);
            cycleMin = Math.min(cycleMin, angle);
            // Only while being pushed up, so encoder jitter near forward can't count as a cycle
            if (power > 0 && lastAngle < 0 && angle >= 0) {
                if (!crossingTimes.isEmpty()) amplitudes.add((cycleMax - cycleMin) / 2);
                crossingTimes.add(t);
                cycleMax = Double.NEGATIVE_INFINITY;
                cycleMin = Double.POSITIVE_INFINITY;
            }
            lastAngle = angle;

            panels.addLine(String.format("Relay test: cycle %d of %d", Math.max(0, crossingTimes.size() - 1), cyclesNeeded));
            panels.addData("Angle (deg)", Math.toDegrees(angle));
            panels.addData("Power", power);
            panels.update(telemetry);
        }
        turret.setPower(0);
        if (!opModeIsActive()) return;

        // Average over the measured cycles only
        double periodSum = 0, amplitudeSum = 0;
        for (int i = WARMUP_CYCLES; i < cyclesNeeded; i++) {
            periodSum += crossingTimes.get(i + 1) - crossingTimes.get(i);
            amplitudeSum += amplitudes.get(i);
        }
        double tu = periodSum / MEASURE_CYCLES;
        double amplitude = amplitudeSum / MEASURE_CYCLES;
        // Describing-function estimate of the ultimate gain, corrected for the hysteresis band
        double ku = 4 * RELAY_POWER / (Math.PI * Math.sqrt(Math.max(amplitude * amplitude - hysteresis * hysteresis, 1e-9)));

        // Gains are in the turret loop's units: power per radian, per radian-second, per radian/sec
        double gentleP = 0.2 * ku, gentleI = 0.4 * ku / tu, gentleD = 0.0667 * ku * tu;
        double classicP = 0.6 * ku, classicI = 1.2 * ku / tu, classicD = 0.075 * ku * tu;

        finish(String.format("Swing ±%.2f°, period %.3f s  ->  Ku %.3f, Tu %.3f s", Math.toDegrees(amplitude), tu, ku, tu),
                String.format("Classic Ziegler-Nichols (faster, ~25%% overshoot): P %.4f  I %.4f  D %.4f", classicP, classicI, classicD),
                "No-overshoot gains, paste into RobotConstants.Launcher:",
                String.format("public static double TURRET_KP = %.4f;", gentleP),
                String.format("public static double TURRET_KI = %.4f;", gentleI),
                String.format("public static double TURRET_KD = %.4f;", gentleD));
    }

    /** Leave the turret off and keep the result on screen until stop. */
    private void finish(String... lines) {
        turret.setPower(0);
        while (opModeIsActive()) {
            for (String line : lines) panels.addLine(line);
            panels.update(telemetry);
            sleep(100);
        }
    }
}
