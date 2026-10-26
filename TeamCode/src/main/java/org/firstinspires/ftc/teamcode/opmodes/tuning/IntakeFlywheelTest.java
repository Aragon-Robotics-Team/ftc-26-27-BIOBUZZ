package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.RobotConstants;

/**
 * Temporary: spin the intake and the flywheel together at one power, like the Motor Tester but for both at once.
 * Gamepad 1: hold RB = both forward, hold LB = both reverse, let go = stop. D-pad up / down = power up / down by
 * powerStep. Power is also editable under Configurables > IntakeFlywheelTest.
 *
 * <p>Raw power (no flywheel PIDF), with each motor's direction from RobotConstants so forward = intake in / shoot.
 * Both coast when stopped, like their subsystems. A motor missing from the config is skipped.
 */
@Configurable
@TeleOp(name = "Intake + Flywheel Test", group = "Tuning")
public class IntakeFlywheelTest extends OpMode {
    public static double power = 0.5;
    public static double powerStep = 0.05;

    private DcMotorEx intake;
    private DcMotorEx flywheel;
    private TelemetryManager panels;

    @Override
    public void init() {
        intake = setUp(RobotConstants.Intake.MOTOR_NAME, RobotConstants.Intake.DIRECTION);
        flywheel = setUp(RobotConstants.Launcher.FLYWHEEL_MOTOR_NAME, RobotConstants.Launcher.FLYWHEEL_DIRECTION);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /** The motor, set up for raw power, or null if it isn't in the active config. */
    private DcMotorEx setUp(String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
        if (motor == null) return null;
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) power += powerStep;
        if (gamepad1.dpadDownWasPressed()) power -= powerStep;
        power = Range.clip(power, 0, 1);

        double output = 0;
        if (gamepad1.right_bumper) output = power;
        else if (gamepad1.left_bumper) output = -power;
        setPower(output);

        panels.addLine("== INTAKE + FLYWHEEL TEST (gamepad 1) ==");
        panels.addLine("Hold RB: both forward   Hold LB: both reverse");
        panels.addLine("D-pad UP / DOWN: more / less power");
        panels.addLine("");
        panels.addData("Power setting", String.format("%.2f", power));
        show("Intake", RobotConstants.Intake.MOTOR_NAME, intake);
        show("Flywheel", RobotConstants.Launcher.FLYWHEEL_MOTOR_NAME, flywheel);
        panels.update(telemetry);
    }

    private void setPower(double output) {
        if (intake != null) intake.setPower(output);
        if (flywheel != null) flywheel.setPower(output);
    }

    private void show(String label, String name, DcMotorEx motor) {
        if (motor == null) {
            panels.addLine(label + ": \"" + name + "\" isn't in the robot config, skipped");
            return;
        }
        panels.addData(label + " output", String.format("%.2f", motor.getPower()));
        panels.addData(label + " velocity (ticks/s)", String.format("%.0f", motor.getVelocity()));
        panels.addData(label + " current (A)", String.format("%.2f", motor.getCurrent(CurrentUnit.AMPS)));
    }

    @Override
    public void stop() {
        setPower(0);
    }
}
