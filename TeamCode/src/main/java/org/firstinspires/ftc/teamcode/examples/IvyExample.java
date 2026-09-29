package org.firstinspires.ftc.teamcode.examples;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_GOAL_POSE;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Sample auto built from Ivy commands instead of a hand-written state machine.
 * Drives to the launch spot, shoots, grabs more game pieces, shoots again, and parks.
 *
 * Ivy basics:
 * - A Command has start / execute / done / end. Commands.instant runs once, waitMs / waitUntil wait,
 *   PedroCommands.follow drives a path and finishes at the end of it.
 * - Subsystems hand out their own commands (launcher.spinUp(), intake.intake(), ...).
 * - Groups combine commands: sequential (one after another), parallel (all at once, done when all are done),
 *   deadline (all at once, done when the first one is done).
 * - Scheduler.schedule starts a command; Scheduler.execute must run every loop.
 */
@Autonomous(name = "Ivy Example", group = "Examples")
public class IvyExample extends OpMode {
    private Robot robot;
    private Follower follower;
    private Launcher launcher;
    private Intake intake;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses (red side, blue is mirrored in start())
    // TODO: real poses
    private final Pose redStartPose = poseFactory.of(0, 0, 0);
    private final Pose redLaunchPose = poseFactory.of(24, 0, 0);
    private final Pose redPickupPose = poseFactory.of(24, 24, 90);
    private final Pose redParkPose = poseFactory.of(0, 24, 0);

    private Pose startPose;
    private Pose launchPose;
    private Pose pickupPose;
    private Pose parkPose;
    private Pose goalPose;

    // Path methods
    private Path startToLaunch() {
        return line(startPose, launchPose).linear(startPose, launchPose);
    }

    private Path launchToPickup() {
        return line(launchPose, pickupPose).linear(launchPose, pickupPose);
    }

    private Path pickupToLaunch() {
        return line(pickupPose, launchPose).linear(pickupPose, launchPose);
    }

    private Path park() {
        return line(launchPose, parkPose).linear(launchPose, parkPose);
    }

    private Command autoRoutine() {
        Command routine = sequential(
                // Spin up while driving so the flywheel is ready when we arrive
                parallel(
                        launcher.spinUp(),
                        follow(follower, startToLaunch())
                ),
                launcher.shoot(),

                // Run the intake on the way to the pickup, stop it on the way back
                intake.intake(),
                follow(follower, launchToPickup()),
                waitMs(300),
                intake.off(),
                follow(follower, pickupToLaunch()),
                launcher.shoot(),

                launcher.idle(),
                follow(follower, park())
        );

        // Keep the turret pointed at the goal for as long as the routine runs
        return deadline(
                routine,
                launcher.aimAt(follower::pose, goalPose)
        );
    }

    @Override
    public void init() {
        // The scheduler is static, so clear anything left over from the last OpMode
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();
        launcher = robot.launcher;
        intake = robot.intake;
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        // Alliance is picked in init_loop, so poses are set here instead of init()
        Alliance alliance = MatchState.alliance;
        startPose = alliance.apply(redStartPose);
        launchPose = alliance.apply(redLaunchPose);
        pickupPose = alliance.apply(redPickupPose);
        parkPose = alliance.apply(redParkPose);
        goalPose = alliance.apply(RED_GOAL_POSE);
        follower.setPose(startPose);

        robot.start();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        robot.update(); // updates the follower
        Scheduler.execute();

        MatchState.pose = follower.pose();
        MatchState.turretAngle = robot.launcher.getTurretAngle();

        telemetry.addData("Alliance", MatchState.alliance);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }
}
