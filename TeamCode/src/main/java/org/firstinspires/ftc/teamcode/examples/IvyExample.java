package org.firstinspires.ftc.teamcode.examples;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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
 * - Groups combine commands: sequential (one after another), parallel (all at once, done when all are done),
 *   deadline (all at once, done when the first one is done).
 * - Scheduler.schedule starts a command; Scheduler.execute must run every loop.
 */
@Autonomous(name = "Ivy Example", group = "Examples")
public class IvyExample extends OpMode {
    // TODO: real poses (red side, blue is mirrored)
    private static final Pose START_POSE = new Pose(0, 0, 0);
    private static final Pose LAUNCH_POSE = new Pose(24, 0, 0);
    private static final Pose PICKUP_POSE = new Pose(24, 24, Math.PI / 2);
    private static final Pose PARK_POSE = new Pose(0, 24, 0);
    private static final Pose GOAL_POSE = new Pose(72, 72, 0); // what the turret aims at

    private Robot robot;

    @Override
    public void init() {
        // The scheduler is static, so clear anything left over from the last OpMode
        Scheduler.reset();
        robot = new Robot(hardwareMap);
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        Alliance alliance = MatchState.alliance;
        robot.drivebase.setPose(alliance.apply(START_POSE));
        robot.start();
        Scheduler.schedule(buildAuto(alliance));
    }

    @Override
    public void loop() {
        Scheduler.execute(); // run commands first so they can set subsystem targets
        robot.update();      // then push targets to hardware (this also updates the Pedro follower)

        MatchState.pose = robot.drivebase.getPose();
        MatchState.turretAngle = robot.launcher.getTurretAngle();

        telemetry.addData("Alliance", MatchState.alliance);
        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }

    private Command buildAuto(Alliance alliance) {
        Pose start = alliance.apply(START_POSE);
        Pose launch = alliance.apply(LAUNCH_POSE);
        Pose pickup = alliance.apply(PICKUP_POSE);
        Pose park = alliance.apply(PARK_POSE);
        Pose goal = alliance.apply(GOAL_POSE);

        Follower follower = robot.drivebase.getFollower();
        Launcher launcher = robot.launcher;
        Intake intake = robot.intake;

        Path toLaunch = line(start, launch).linear(start, launch);
        Path toPickup = line(launch, pickup).linear(launch, pickup);
        Path backToLaunch = line(pickup, launch).linear(pickup, launch);
        Path toPark = line(launch, park).linear(launch, park);

        Command routine = sequential(
                // Spin up while driving so the flywheel is ready when we arrive
                parallel(
                        instant(launcher::spinUp),
                        follow(follower, toLaunch)
                ),
                shoot(launcher),

                // Run the intake on the way to the pickup, stop it on the way back
                instant(intake::intake),
                follow(follower, toPickup),
                waitMs(300),
                instant(intake::off),
                follow(follower, backToLaunch),
                shoot(launcher),

                instant(launcher::idle),
                follow(follower, toPark)
        );

        // Keep the turret pointed at the goal for as long as the routine runs
        return deadline(
                routine,
                infinite(() -> launcher.aimAt(robot.drivebase.getPose(), goal))
        );
    }

    /** Wait until the launcher is ready (max 1.5 s so a slow flywheel can't stall the auto), then fire. */
    private Command shoot(Launcher launcher) {
        return sequential(
                waitUntil(launcher::isReady).raceWith(waitMs(1500)),
                instant(launcher::launch),
                waitMs(250) // TODO: tune shot delay
        );
    }
}
