package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_GOAL_POSE;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        Scheduler.reset();

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
        // Always-on commands, running for the whole match
        schedule(
                // TODO: slow mode
                robot.drivebase.drive(
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x
                ),
                robot.launcher.aimAt(robot.drivebase::getPose, MatchState.alliance.apply(RED_GOAL_POSE))
        );
    }

    @Override
    public void loop() {
        bindControls();

        robot.update();
        Scheduler.execute();

        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }

    /** Button presses schedule commands. TODO: confirm the layout with the drivers. */
    private void bindControls() {
        // Hold right bumper to intake, left bumper to outtake
        if (gamepad1.rightBumperWasPressed()) schedule(robot.intake.intake());
        if (gamepad1.leftBumperWasPressed()) schedule(robot.intake.outtake());
        if (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased()) schedule(robot.intake.off());

        if (gamepad1.yWasPressed()) schedule(robot.launcher.toggleFlywheel());
        if (gamepad1.aWasPressed()) schedule(robot.launcher.shoot());
    }
}
