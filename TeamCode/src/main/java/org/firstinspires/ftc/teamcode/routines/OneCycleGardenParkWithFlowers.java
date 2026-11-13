package org.firstinspires.ftc.teamcode.routines;

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
 * 1 Cycle + Garden + Flowers + Park: {@link OneCycleGardenPark}, then instead of parking straight away, knock the
 * pollen out of the left (alliance-wall) flower and the far (rear-wall) flower by sliding in and out against each,
 * then park on the top side of the loading zone with the intake still running.
 */
public class OneCycleGardenParkWithFlowers extends AutoRoutine {
    /**
     * Partway to the far flower, so the drive there stays clear of the end of our hive's foot (a straight line from
     * redLeftFlowerBackout to the far flower would brush it). Only this routine uses it, so it lives here; it's the
     * old rear-flower approach spot (Nov 10), already facing the rear wall.
     */
    private final Pose farFlowerAdjust = poseFactory.of(34, 118.764, 90);

    public OneCycleGardenParkWithFlowers(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }

    @Override
    public Pose startPose() {
        return poses.start;
    }

    // Path methods
    private Path startToLaunch() {
        return line(poses.start, poses.startLaunch).linear(poses.start, poses.startLaunch);
    }

    private Path prepareGardenIntake() {
        return line(poses.startLaunch, poses.gardenBorder).linear(poses.startLaunch, poses.gardenBorder);
    }

    private Path gardenIntake1() {
        return line(poses.gardenBorder, poses.gardenIntake).linear(poses.gardenBorder, poses.gardenIntake);
    }

    private Path gardenIntakeBack() {
        return line(poses.gardenIntake, poses.gardenIntakeBack).linear(poses.gardenIntake, poses.gardenIntakeBack);
    }

    private Path gardenIntake2() {
        return line(poses.gardenIntakeBack, poses.gardenIntake).linear(poses.gardenIntakeBack, poses.gardenIntake);
    }

    private Path adjustBeforePark() {
        return line(poses.gardenIntake, poses.adjustToPark).linear(poses.gardenIntake, poses.adjustToPark);
    }

    // Left flower: arrive at the Out spot facing the wall, slide In / Out / In, then back off the wall
    private Path toLeftFlower() {
        return line(poses.adjustToPark, poses.redLeftFlowerOut).linear(poses.adjustToPark, poses.redLeftFlowerOut);
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

    // Far flower: around the hive's foot to the adjust spot, Out, In, Out, In, then back off the wall
    private Path toFarFlowerAdjust() {
        return line(poses.redLeftFlowerBackout, farFlowerAdjust).linear(poses.redLeftFlowerBackout, farFlowerAdjust);
    }

    private Path toFarFlower() {
        return line(farFlowerAdjust, poses.redFarFlowerOut).linear(farFlowerAdjust, poses.redFarFlowerOut);
    }

    private Path farFlowerIn() {
        return line(poses.redFarFlowerOut, poses.redFarFlowerIn).linear(poses.redFarFlowerOut, poses.redFarFlowerIn);
    }

    private Path farFlowerOut() {
        return line(poses.redFarFlowerIn, poses.redFarFlowerOut).linear(poses.redFarFlowerIn, poses.redFarFlowerOut);
    }

    private Path farFlowerBackout() {
        return line(poses.redFarFlowerIn, poses.redFarFlowerBackout).linear(poses.redFarFlowerIn, poses.redFarFlowerBackout);
    }

    private Path park() {
        return line(poses.redFarFlowerBackout, poses.topPark).linear(poses.redFarFlowerBackout, poses.topPark);
    }

    @Override
    public Command autoRoutine() {
        Command cycle = sequential(
                // Preloads
                robot.intake.intake(),
                follow(follower, startToLaunch()),
                waitMs(2000),
                robot.shoot(),
                // Tip
                robot.flipHive(),

                // Garden: drive in twice, backing up between, so the corner ball can roll out for the second try
                follow(follower, prepareGardenIntake()),
                follow(follower, gardenIntake1()).raceWith(waitMs(1500)),
                waitMs(1000),
                follow(follower, gardenIntakeBack()),
                follow(follower, gardenIntake2()).raceWith(waitMs(1500)),
                waitMs(1000),
                follow(follower, adjustBeforePark()),

                // Left flower: slide in and out against it to knock its pollen out (intake still on)
                follow(follower, toLeftFlower()),
                follow(follower, leftFlowerIn()).raceWith(waitMs(1500)),
                waitMs(500),
                follow(follower, leftFlowerOut()),
                follow(follower, leftFlowerIn()).raceWith(waitMs(1500)),
                waitMs(500),
                follow(follower, leftFlowerBackout()),

                // Far flower: arrive at the Out spot, slide In / Out / In, then back off the wall
                follow(follower, toFarFlowerAdjust()),
                follow(follower, toFarFlower()),
                follow(follower, farFlowerIn()).raceWith(waitMs(1500)),
                waitMs(500),
                follow(follower, farFlowerOut()),
                follow(follower, farFlowerIn()).raceWith(waitMs(1500)),
                waitMs(500),
                follow(follower, farFlowerBackout())
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
