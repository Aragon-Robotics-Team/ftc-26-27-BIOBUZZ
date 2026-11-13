package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Robot;

/**
 * 1 Cycle + Garden + Park (doc/auto-pathing.md, doc/autoPP/red_1cycle_park.pp): shoot the preloads, collect the
 * garden pollen, then park on the bottom side of the loading zone with it still on board.
 */
public class OneCycleGardenParkWithM extends AutoRoutine {
    public OneCycleGardenParkWithM(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }

    @Override
    public Pose startPose() {
        return poses.topPark;
    }

    // Path methods
    private Path startToFlowerReady() {
        return line(poses.topPark, poses.flowerReady).linear(poses.topPark, poses.flowerReady);
    }

    private Path flowerKnock() {
        return line(poses.flowerReady, poses.flowerKnock).linear(poses.startLaunch, poses.gardenBorder);
    }

    private Path flowerBack() {
        return line(poses.flowerKnock, poses.flowerReady).linear(poses.flowerKnock, poses.flowerReady);
    }

    private Path flowerKnockExit() {
        return line(poses.flowerReady, poses.flowerOut).reverseTangent();
    }

    private Path adjustBeforeShoot() {
        return line(poses.flowerOut, poses.adjustToPark);
    }
    private Path outToShoot() {
        return line(poses.adjustToPark, poses.startLaunch).linear(poses.adjustToPark, poses.startLaunch);
    }

    private Path adjustBeforePark() {
        return line(poses.startLaunch, poses.adjustToPark).linear(poses.startLaunch, poses.adjustToPark);
    }

    private Path park() {
        return line(poses.adjustToPark, poses.bottomPark).linear(poses.adjustToPark, poses.bottomPark);
    }

    @Override
    public Command autoRoutine() {
        Command cycle = sequential(
                // Preloads
                waitMs(15000),
                robot.intake.intake(),
                follow(follower, startToFlowerReady()),
                follow(follower, flowerKnock()).raceWith(waitMs(1500)),
                waitMs(1000),
                follow(follower, flowerBack()),
                follow(follower, flowerKnock()).raceWith(waitMs(1500)),
                waitMs(1000),
                follow(follower, flowerBack()),
                follow(follower, flowerKnockExit()),
                follow(follower, adjustBeforeShoot()),
                follow(follower, outToShoot()),
                waitMs(1000),
                robot.shoot(),
                follow(follower, adjustBeforePark())
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(cycle, robot.aimAtHive(), robot.enableFlywheel()),
                follow(follower, park()),
                waitMs(3000),
                robot.intake.off()
        );
    }
}
