package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Tune the turret PID. Start with the turret facing forward (that's where it zeroes).
 * 1. Raise P until the turret gets to the target quickly, then back off if it overshoots.
 * 2. Add D to damp any overshoot. Add I only if it stalls just short of the target.
 * X / A / B jump the target left / center / right by the step angle, so you can watch the response.
 */
@TeleOp(name = "Turret Tuner", group = "Tuning")
public class TurretTuner extends OpMode {
    private Launcher launcher;
    private TuningMenu menu;
    private double stepDegrees = 45;

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);
        menu = new TuningMenu()
                .add("Step (deg)", 5, () -> stepDegrees, v -> stepDegrees = v)
                .add("P", 0.1, () -> RobotConstants.Launcher.TURRET_KP, v -> RobotConstants.Launcher.TURRET_KP = v)
                .add("I", 0.01, () -> RobotConstants.Launcher.TURRET_KI, v -> RobotConstants.Launcher.TURRET_KI = v)
                .add("D", 0.01, () -> RobotConstants.Launcher.TURRET_KD, v -> RobotConstants.Launcher.TURRET_KD = v)
                .add("Max power", 0.05,
                        () -> RobotConstants.Launcher.TURRET_MAX_POWER, v -> RobotConstants.Launcher.TURRET_MAX_POWER = v);
    }

    @Override
    public void start() {
        launcher.start();
    }

    @Override
    public void loop() {
        menu.update(gamepad1);
        // Positive angle is counter-clockwise, i.e. to the left
        if (gamepad1.xWasPressed()) launcher.setTurretAngle(Math.toRadians(stepDegrees));
        if (gamepad1.aWasPressed()) launcher.setTurretAngle(0);
        if (gamepad1.bWasPressed()) launcher.setTurretAngle(Math.toRadians(-stepDegrees));

        launcher.update();
        telemetry.addLine("X left, A center, B right");
        telemetry.addData("On target", launcher.isTurretOnTarget());
        launcher.telemetry(telemetry);
        menu.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.stop();
    }
}
