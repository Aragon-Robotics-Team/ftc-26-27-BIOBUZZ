package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.RobotConstants.Drive.SLOW_MODE_SCALE;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_RIGHT_CORNER;
import static org.firstinspires.ftc.teamcode.util.Html.bold;
import static org.firstinspires.ftc.teamcode.util.Html.color;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Hive;
import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.util.Html;

/**
 * Gamepad 1 (driver):
 *   sticks          field-centric drive (stick forward = away from our driver station)
 *   hold LT         slow mode
 *   triangle        toggle intake        cross   toggle outtake
 *   RB              flywheel on          LB      flywheel off
 *   hold RT         open the gate and feed the shooter
 * Gamepad 2 (operator):
 *   dpad left/right nudge turret         dpad up/down  nudge flywheel speed
 *   square / circle hive LEFT / RIGHT cell is up
 *   share / options reset pose: robot pushed into our LEFT / RIGHT corner, back to the alliance wall
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp extends OpMode {
    private Robot robot;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        // Pick up where Auto left off. Without an Auto run the robot starts at the origin with the turret
        // forward; push it into a corner and use share / options to localize.
        if (MatchState.pose != null) {
            robot.drivebase.setPose(MatchState.pose);
            robot.launcher.setCurrentTurretAngle(MatchState.turretAngle);
        }
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        telemetry.addData("Hive", MatchState.hive);
        telemetry.addData("Start pose", MatchState.pose != null ? MatchState.pose : "none (Auto didn't run)");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.start();
        // Stick forward points away from our driver station: +x for red, -x for blue
        double headingOffset = MatchState.alliance == Alliance.BLUE ? Math.PI : 0;
        // Always-on commands, running for the whole match
        schedule(
                robot.drivebase.driveFieldCentric(
                        () -> -gamepad1.left_stick_y * driveScale(),
                        () -> -gamepad1.left_stick_x * driveScale(),
                        () -> -gamepad1.right_stick_x * driveScale(),
                        headingOffset
                ),
                robot.aimAtHive()
        );
        loopTimer.reset();
    }

    @Override
    public void loop() {
        bindControls();

        robot.update();
        Scheduler.execute();

        showTelemetry();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }

    private double driveScale() {
        return gamepad1.left_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
    }

    /** Button presses schedule commands. */
    private void bindControls() {
        // ---- Gamepad 1: driver ----
        if (gamepad1.triangleWasPressed()) schedule(robot.intake.toggleIntake());
        if (gamepad1.crossWasPressed()) schedule(robot.intake.toggleOuttake());

        if (gamepad1.rightBumperWasPressed()) schedule(robot.enableFlywheel());
        if (gamepad1.leftBumperWasPressed()) schedule(robot.launcher.idle());

        if (gamepad1.rightTriggerWasPressed()) schedule(robot.startShooting());
        if (gamepad1.rightTriggerWasReleased()) schedule(robot.stopShooting());

        // ---- Gamepad 2: operator (manual corrections) ----
        if (gamepad2.dpadLeftWasPressed()) schedule(robot.launcher.nudgeTurret(+1));
        if (gamepad2.dpadRightWasPressed()) schedule(robot.launcher.nudgeTurret(-1));
        if (gamepad2.dpadUpWasPressed()) schedule(robot.launcher.nudgeVelocity(+1));
        if (gamepad2.dpadDownWasPressed()) schedule(robot.launcher.nudgeVelocity(-1));

        if (gamepad2.squareWasPressed()) schedule(robot.setHive(Hive.LEFT));
        if (gamepad2.circleWasPressed()) schedule(robot.setHive(Hive.RIGHT));

        if (gamepad2.shareWasPressed()) schedule(robot.resetPose(RED_LEFT_CORNER));
        if (gamepad2.optionsWasPressed()) schedule(robot.resetPose(RED_RIGHT_CORNER));
    }

    /** The first lines are what the drivers need mid-match; details are below the fold. */
    private void showTelemetry() {
        Launcher launcher = robot.launcher;
        Pose pose = robot.drivebase.getPose();

        telemetry.addLine(Html.alliance(MatchState.alliance) + bold(" · Hive: " + MatchState.hive));
        telemetry.addLine(String.format("Flywheel %s  %.0f / %.0f  (%+.0f)",
                flywheelStatus(launcher), launcher.getFlywheelVelocity(), launcher.getTargetVelocity(),
                launcher.getVelocityNudge()));
        telemetry.addLine(String.format("Turret %s  %.1f° / %.1f°  (%+.0f°)",
                turretStatus(launcher),
                Math.toDegrees(launcher.getTurretTargetAngle()), Math.toDegrees(launcher.getTurretAngle()),
                Math.toDegrees(launcher.getTurretNudge())));
        telemetry.addLine(String.format("Distance to hive  %.1f in", robot.distanceToHive()));
        telemetry.addLine(String.format("Pose  x %.1f  y %.1f  h %.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addLine("Intake " + robot.intake.getMode() + (robot.intake.isFeeding() ? " (FEEDING)" : "")
                + " · Gate " + (robot.gate.isOpen() ? "OPEN" : "CLOSED"));
        telemetry.addLine(String.format("Loop  %.0f ms", loopTimer.milliseconds()));
        loopTimer.reset();

        telemetry.addLine(color("──── details ────", Html.GRAY));
        telemetry.addData("Follower mode", robot.drivebase.getFollower().mode());
        robot.telemetry(telemetry);
        telemetry.update();
    }

    private static String turretStatus(Launcher launcher) {
        if (!launcher.hasShot()) return color("NO SHOT", Html.RED);
        return launcher.isTurretOnTarget() ? color("ON TARGET", Html.GREEN) : color("AIMING", Html.YELLOW);
    }

        private static String flywheelStatus(Launcher launcher) {
        if (!launcher.isSpinning()) return bold("OFF");
        if (launcher.isFlywheelAtSpeed()) return bold(color("READY", Html.GREEN));
        return launcher.getFlywheelVelocity() < launcher.getTargetVelocity()
                ? bold(color("UNDER", Html.YELLOW))
                : bold(color("OVER", Html.RED));
    }
}
