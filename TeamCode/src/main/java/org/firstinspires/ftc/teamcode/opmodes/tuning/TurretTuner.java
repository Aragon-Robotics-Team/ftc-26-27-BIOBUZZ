package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Tune the turret PID with Panels: edit the values under Configurables > TurretTuner and watch
 * target / actual in the Graph. Copy the final values into RobotConstants.Launcher.
 * Start with the turret facing forward (that's where it zeroes).
 * 0. Set kS (from the Turret Auto Tuner) so small errors still move the turret past friction.
 * 1. Raise kP until the turret gets to the target quickly, then back off if it overshoots.
 * 2. Add kD to damp any overshoot. Add kI only if it stalls just short of the target.
 * Square / cross / circle (gamepad 1) jump the target left / center / right by stepDegrees, so you can watch the
 * response.
 */
@Configurable
@TeleOp(name = "Turret Tuner", group = "Tuning")
public class TurretTuner extends OpMode {
    public static double stepDegrees = 45;
    public static double kP = RobotConstants.Launcher.TURRET_KP;
    public static double kI = RobotConstants.Launcher.TURRET_KI;
    public static double kD = RobotConstants.Launcher.TURRET_KD;
    public static double kS = RobotConstants.Launcher.TURRET_KS;
    public static double maxPower = RobotConstants.Launcher.TURRET_MAX_POWER;

    private Launcher launcher;
    private TelemetryManager panels;

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
        // The launcher reads these every loop, so edits in Panels take effect right away
        RobotConstants.Launcher.TURRET_KP = kP;
        RobotConstants.Launcher.TURRET_KI = kI;
        RobotConstants.Launcher.TURRET_KD = kD;
        RobotConstants.Launcher.TURRET_KS = kS;
        RobotConstants.Launcher.TURRET_MAX_POWER = maxPower;

        // Positive angle is counter-clockwise, i.e. to the left
        if (gamepad1.squareWasPressed()) schedule(launcher.turnTurretTo(Math.toRadians(stepDegrees)));
        if (gamepad1.crossWasPressed()) schedule(launcher.turnTurretTo(0));
        if (gamepad1.circleWasPressed()) schedule(launcher.turnTurretTo(Math.toRadians(-stepDegrees)));

        launcher.update();
        Scheduler.execute();

        double target = Math.toDegrees(launcher.getTurretTargetAngle());
        double actual = Math.toDegrees(launcher.getTurretAngle());
        panels.addLine("Square left, cross center, circle right");
        panels.addData("On target", launcher.isTurretOnTarget());
        panels.addData("Target (deg)", target);
        panels.addData("Actual (deg)", actual);
        panels.addData("Error (deg)", target - actual);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        Scheduler.reset();
        launcher.stop();
    }
}
