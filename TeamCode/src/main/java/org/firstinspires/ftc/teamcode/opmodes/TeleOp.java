package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        // Pick up where Auto left off. Without an Auto run the robot starts at the origin with the turret forward.
        if (MatchState.pose != null) {
            robot.drivebase.setPose(MatchState.pose);
            robot.launcher.setCurrentTurretAngle(MatchState.turretAngle);
        }
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        telemetry.addData("Start pose", MatchState.pose != null ? MatchState.pose : "none (Auto didn't run)");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
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
        // TODO: slow mode
        robot.drivebase.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }

    private void handleIntake() {
        // TODO: intake
    }

    private void handleLauncher() {
        // TODO: bind launcher controls
        // TODO: aim turret at goal
    }
}
