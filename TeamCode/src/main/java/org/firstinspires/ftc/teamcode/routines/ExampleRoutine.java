package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Sample routine showing Ivy with the robot's mechanisms. Disabled so it never shows up in the Auto menu;
 * copy it as a starting point for a real routine.
 * Drives to the launch spot, shoots (tipping the hive), grabs more game pieces, shoots again, and parks.
 *
 * Ivy basics:
 * - A Command has start / execute / done / end. Commands.instant runs once, waitMs / waitUntil wait,
 *   PedroCommands.follow drives a path and finishes at the end of it.
 * - Subsystems hand out their own commands (robot.intake.intake(), robot.launcher.idle(), ...);
 *   Robot has the ones that use several subsystems (robot.shoot(), robot.aimAtHive(), ...).
 * - Groups combine commands: sequential (one after another), parallel (all at once, done when all are done),
 *   deadline (all at once, done when the first one is done).
 */
@Disabled
public class ExampleRoutine extends AutoRoutine {
    // Poses (red side, rotated 180° for blue)
    // TODO: real poses
    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose launchPose = poseFactory.of(24, 0, 0);
    private final Pose pickupPose = poseFactory.of(24, 24, 90);
    private final Pose parkPose = poseFactory.of(0, 24, 0);

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
        Command cycles = sequential(
                follow(follower, startToLaunch()),
                robot.shoot(),
                // The preloads tip the hive, so the other cell is up now
                robot.flipHive(),

                // Run the intake on the way to the pickup, stop it on the way back
                robot.intake.intake(),
                follow(follower, launchToPickup()),
                waitMs(300),
                robot.intake.off(),
                follow(follower, pickupToLaunch()),
                robot.shoot()
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(cycles, robot.aimAtHive(), robot.enableFlywheel()),
                robot.launcher.idle(),
                follow(follower, park())
        );
    }
}
