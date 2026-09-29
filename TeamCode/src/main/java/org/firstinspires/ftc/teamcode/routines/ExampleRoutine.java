package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_GOAL_POSE;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Sample routine showing Ivy with the robot's mechanisms.
 * Drives to the launch spot, shoots, grabs more game pieces, shoots again, and parks.
 *
 * Ivy basics:
 * - A Command has start / execute / done / end. Commands.instant runs once, waitMs / waitUntil wait,
 *   PedroCommands.follow drives a path and finishes at the end of it.
 * - Subsystems hand out their own commands (launcher.spinUp(), intake.intake(), ...).
 * - Groups combine commands: sequential (one after another), parallel (all at once, done when all are done),
 *   deadline (all at once, done when the first one is done).
 */
public class ExampleRoutine extends AutoRoutine {
    private final Launcher launcher = robot.launcher;
    private final Intake intake = robot.intake;

    // Poses (red side, rotated 180° for blue)
    // TODO: real poses
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose launchPose = poseFactory.of(24, 0, 0);
    private final Pose pickupPose = poseFactory.of(24, 24, 90);
    private final Pose parkPose = poseFactory.of(0, 24, 0);
    private final Pose goalPose = alliance.apply(RED_GOAL_POSE);

    public ExampleRoutine(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }

    @Override
    public Pose startPose() {
        return startPose;
    }

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

    @Override
    public Command autoRoutine() {
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
}
