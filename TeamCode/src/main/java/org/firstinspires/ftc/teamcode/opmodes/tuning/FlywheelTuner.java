package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Tune the flywheel velocity PIDF with Panels: edit the values under Configurables > FlywheelTuner and
 * watch target / actual in the Graph. Copy the final values into RobotConstants.Launcher.
 * 1. With P = I = D = 0, raise F until the actual speed sits just under the target (or use F ≈ 32767 / max ticks/sec).
 * 2. Raise P until it recovers quickly from a shot without oscillating. Add I only if it settles below target.
 * A (gamepad 1) toggles the flywheel on / off.
 */
@Configurable
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends OpMode {
    public static double targetVelocity = RobotConstants.Launcher.TARGET_VELOCITY; // ticks/sec
    public static double p = RobotConstants.Launcher.FLYWHEEL_PIDF.p;
    public static double i = RobotConstants.Launcher.FLYWHEEL_PIDF.i;
    public static double d = RobotConstants.Launcher.FLYWHEEL_PIDF.d;
    public static double f = RobotConstants.Launcher.FLYWHEEL_PIDF.f;

    private Launcher launcher;
    private TelemetryManager panels;
    private final PIDFCoefficients pushedPidf = new PIDFCoefficients();
    private double pushedTarget = Double.NaN;

    @Override
    public void init() {
        Scheduler.reset();

        launcher = new Launcher(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        launcher.start();
    }

    @Override
    public void loop() {
        pushChanges();
        if (gamepad1.aWasPressed()) schedule(launcher.toggleFlywheel());

        launcher.update();
        Scheduler.execute();

        panels.addLine(launcher.isSpinning() ? "Flywheel ON (A to stop)" : "Flywheel OFF (A to start)");
        panels.addData("Target", launcher.getTargetVelocity());
        panels.addData("Actual", launcher.getFlywheelVelocity());
        panels.addData("Error", launcher.getTargetVelocity() - launcher.getFlywheelVelocity());
        panels.update(telemetry);
    }

    /** Send any values edited in Panels to the launcher. */
    private void pushChanges() {
        if (p != pushedPidf.p || i != pushedPidf.i || d != pushedPidf.d || f != pushedPidf.f) {
            pushedPidf.p = p;
            pushedPidf.i = i;
            pushedPidf.d = d;
            pushedPidf.f = f;
            launcher.setFlywheelPIDF(pushedPidf);
        }
        if (targetVelocity != pushedTarget) {
            pushedTarget = targetVelocity;
            RobotConstants.Launcher.TARGET_VELOCITY = targetVelocity;
            if (launcher.isSpinning()) schedule(launcher.spinUp()); // pick up the new target
        }
    }

    @Override
    public void stop() {
        Scheduler.reset();
        launcher.stop();
    }
}
