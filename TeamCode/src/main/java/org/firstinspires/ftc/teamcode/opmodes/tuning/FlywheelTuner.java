package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.FLYWHEEL_PIDF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Tune the flywheel velocity PIDF.
 * 1. With P = I = D = 0, raise F until the actual speed sits just under the target (or use F ≈ 32767 / max ticks/sec).
 * 2. Raise P until it recovers quickly from a shot without oscillating. Add I only if it settles below target.
 * A toggles the flywheel on / off.
 */
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends OpMode {
    private Launcher launcher;
    private TuningMenu menu;
    private boolean spinning = false;

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);
        menu = new TuningMenu()
                .add("Target (ticks/s)", 100,
                        () -> RobotConstants.Launcher.TARGET_VELOCITY, v -> RobotConstants.Launcher.TARGET_VELOCITY = v)
                .add("F", 1, () -> FLYWHEEL_PIDF.f, v -> FLYWHEEL_PIDF.f = v)
                .add("P", 1, () -> FLYWHEEL_PIDF.p, v -> FLYWHEEL_PIDF.p = v)
                .add("I", 0.1, () -> FLYWHEEL_PIDF.i, v -> FLYWHEEL_PIDF.i = v)
                .add("D", 0.1, () -> FLYWHEEL_PIDF.d, v -> FLYWHEEL_PIDF.d = v);
    }

    @Override
    public void start() {
        launcher.start();
    }

    @Override
    public void loop() {
        if (menu.update(gamepad1)) {
            launcher.setFlywheelPIDF(FLYWHEEL_PIDF);
            if (spinning) launcher.spinUp(); // pick up a new target
        }
        if (gamepad1.aWasPressed()) {
            spinning = !spinning;
            if (spinning) launcher.spinUp();
            else launcher.idle();
        }

        launcher.update();
        telemetry.addData("Flywheel", spinning ? "ON (A to stop)" : "OFF (A to start)");
        launcher.telemetry(telemetry);
        menu.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.stop();
    }
}
