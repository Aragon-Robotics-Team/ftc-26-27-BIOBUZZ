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
        return poses.farStart;
    }

    // Path methods
    private Path leftFlowerApproach() {
        return line(poses.farStart, poses.redLeftFlowerBackout).linear(poses.farStart, poses.redLeftFlowerBackout);
    }
    private Path toLeftFlower() {
        return line(poses.redLeftFlowerBackout, poses.redLeftFlowerOut).linear(poses.redLeftFlowerBackout, poses.redLeftFlowerOut);
    }

    private Path leftFlowerIn() {
        return line(poses.redLeftFlowerOut, poses.redLeftFlowerIn).linear(poses.redLeftFlowerOut, poses.redLeftFlowerIn);
    }

    private Path leftFlowerOut() {
        return line(poses.redLeftFlowerIn, poses.redLeftFlowerOut).linear(poses.redLeftFlowerIn, poses.redLeftFlowerOut);
    }

    private Path leftFlowerBackout() {
        return line(poses.redLeftFlowerIn, poses.redLeftFlowerBackout).linear(poses.redLeftFlowerIn, poses.redLeftFlowerBackout);
    }

    private Path adjustBeforeShoot() {
        return line(poses.redLeftFlowerBackout, poses.adjustToPark).linear(poses.redLeftFlowerBackout, poses.adjustToPark);
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
        Command flowers = sequential(
                // Preloads
                robot.launcher.idle(),
                waitMs(15000),
                robot.intake.intake(),
                follow(follower, leftFlowerApproach()),
                follow(follower, toLeftFlower()),
                follow(follower, leftFlowerIn()).raceWith(waitMs(1000)),
                waitMs(500),
                follow(follower, leftFlowerOut()),
                follow(follower, leftFlowerIn()).raceWith(waitMs(1000)),
                waitMs(500),
                follow(follower, leftFlowerOut())
        );

        Command shoot = sequential(
                follow(follower, leftFlowerBackout()),
                follow(follower, adjustBeforeShoot()),
                follow(follower, outToShoot()),
                waitMs(1000),
                robot.shoot(),
                follow(follower, adjustBeforePark())
        );

        return sequential(
                robot.launcher.idle(),
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(sequential(flowers, deadline(shoot, robot.enableFlywheel())), robot.aimAtHive()),
                follow(follower, park()),
                waitMs(2000),
                robot.intake.off(),
                robot.launcher.idle()
        );
    }
}
