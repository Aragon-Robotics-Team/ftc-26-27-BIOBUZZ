package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Robot;

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
public class UnobtrusivePark extends AutoRoutine {

    @Override
    public Pose startPose() {
        return poses.start;
    }
    public Path park() {
        return Paths.line(poses.farStart, poses.topPark).linear(poses.farStart, poses.topPark);
    }

    public UnobtrusivePark(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }





    @Override
    public Command autoRoutine() {
        Command cycles = sequential(
              robot.intake.intake()
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(cycles, robot.aimAtHive(), robot.enableFlywheel()),
                robot.launcher.idle(),
                follow(follower, park()),
                robot.intake.off()
        );
    }
}
