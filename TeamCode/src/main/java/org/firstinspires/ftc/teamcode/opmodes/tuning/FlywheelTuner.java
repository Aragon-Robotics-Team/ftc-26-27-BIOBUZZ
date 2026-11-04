package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.FlywheelControl;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Tune the flywheel speed control with Panels: edit the values under Configurables > FlywheelTuner and
 * watch target / actual / power in the Graph. Copy the final values into RobotConstants.Launcher.
 * The control runs in this OpMode's loop (subsystems.FlywheelController), so it reacts once per loop: the loop time is
 * shown, and TeleOp's is longer, so check recovery there too.
 * PIDF (bangBang = false):
 * 1. With kP = kI = kD = 0, set kV so the actual speed sits just under the target (the Flywheel Auto Tuner measures it).
 * 2. Raise kP until it recovers quickly from a shot without oscillating. Add kI only if it settles below target.
 * Bang-bang (bangBang = true): only kV matters. Lower it if the speed sits above target with the power at the hold
 * value; it's fine for the power to flick to full now and then.
 * Cross (gamepad 1) toggles the flywheel on / off.
 */
@Configurable
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends OpMode {
    public static double targetVelocity = RobotConstants.Launcher.TARGET_VELOCITY; // ticks/sec
    public static boolean bangBang = RobotConstants.Launcher.FLYWHEEL_CONTROL == FlywheelControl.BANG_BANG;
    public static double kV = RobotConstants.Launcher.FLYWHEEL_KV;
    public static double kS = RobotConstants.Launcher.FLYWHEEL_KS;
    public static double kP = RobotConstants.Launcher.FLYWHEEL_KP;
    public static double kI = RobotConstants.Launcher.FLYWHEEL_KI;
    public static double kD = RobotConstants.Launcher.FLYWHEEL_KD;

    private Launcher launcher;
    private TelemetryManager panels;
    private double pushedTarget = Double.NaN;
    private final ElapsedTime loopTimer = new ElapsedTime();

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
        if (gamepad1.crossWasPressed()) schedule(launcher.toggleFlywheel());

        launcher.update();
        Scheduler.execute();

        panels.addLine(launcher.isSpinning() ? "Flywheel ON (cross to stop)" : "Flywheel OFF (cross to start)");
        panels.addLine("Control: " + RobotConstants.Launcher.FLYWHEEL_CONTROL);
        panels.addData("Target", launcher.getTargetVelocity());
        panels.addData("Actual", launcher.getFlywheelVelocity());
        panels.addData("Error", launcher.getTargetVelocity() - launcher.getFlywheelVelocity());
        panels.addData("Power", launcher.getFlywheelPower());
        panels.addData("Loop (ms)", loopTimer.milliseconds());
        loopTimer.reset();
        panels.update(telemetry);
    }

    /** Send any values edited in Panels to the launcher. */
    private void pushChanges() {
        // The flywheel control reads these every loop
        RobotConstants.Launcher.FLYWHEEL_CONTROL = bangBang ? FlywheelControl.BANG_BANG : FlywheelControl.PIDF;
        RobotConstants.Launcher.FLYWHEEL_KV = kV;
        RobotConstants.Launcher.FLYWHEEL_KS = kS;
        RobotConstants.Launcher.FLYWHEEL_KP = kP;
        RobotConstants.Launcher.FLYWHEEL_KI = kI;
        RobotConstants.Launcher.FLYWHEEL_KD = kD;
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
