package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        // TODO: start pose (e.g. handed off from Auto)
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        handleDrive();
        handleIntake();
        handleLauncher();

        robot.update();
        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private void handleDrive() {
        // TODO: slow mode, field-centric toggle, etc.
        robot.drivebase.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }

    private void handleIntake() {
        // TODO: bind controls
    }

    private void handleLauncher() {
        // TODO: bind controls
    }
}
