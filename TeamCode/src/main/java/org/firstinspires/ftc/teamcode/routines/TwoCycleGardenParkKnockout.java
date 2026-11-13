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
 * 2 Cycles (Garden) + Park (doc/auto-pathing.md, doc/autoPP/red_2cycle_park.pp): shoot the preloads, collect the
 * garden pollen, drive up the middle and shoot it from the flower-side launch spot, then park on the top side of the
 * loading zone.
 */
public class TwoCycleGardenParkKnockout extends AutoRoutine {
    public TwoCycleGardenParkKnockout(Robot robot, Alliance alliance) {
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

    private Path gardenToCenterLine() {
        return line(poses.gardenIntake, poses.gardenCenterLine).linear(poses.gardenIntake, poses.gardenCenterLine);
    }

    private Path centerLineToFlowerLaunch() {
        return line(poses.gardenCenterLine, poses.flowerLaunch).linear(poses.gardenCenterLine, poses.flowerLaunch);
    }

    private Path launchToFlower(){
        return line(poses.flowerLaunch, poses.redFarFlowerIn).linear(poses.flowerLaunch, poses.redFarFlowerIn);
    }

    private Path flowerToBack(){
        return line(poses.redFarFlowerIn, poses.redFarFlowerBackout).linear(poses.redFarFlowerIn, poses.redFarFlowerBackout);
    }

    private Path backToFlower(){
        return line(poses.redFarFlowerBackout, poses.redFarFlowerIn).linear(poses.redFarFlowerBackout, poses.redFarFlowerIn);
    }

    private Path park() {
        return line(poses.flowerLaunch, poses.topPark).linear(poses.flowerLaunch, poses.topPark);
    }

    @Override
    public Command autoRoutine() {
        Command cycles = sequential(
                // Preloads
                follow(follower, startToLaunch()),
                robot.shoot(),
                // Tip
                robot.flipHive(),

                // Garden: intake on the way in, stop once out at the center line
                robot.intake.intake(),
                follow(follower, prepareGardenIntake()),
                follow(follower, gardenIntake1()),
                waitMs(1000),
                follow(follower, gardenIntakeBack()),
                follow(follower, gardenIntake2()).raceWith(waitMs(1500)),
                waitMs(1000),
                follow(follower, gardenToCenterLine()),

                // Up the middle to the other side and shoot the garden pollen
                follow(follower, centerLineToFlowerLaunch()),
                robot.shoot(),
                // Second tip
                robot.flipHive(),

                follow(follower,launchToFlower()).raceWith(waitMs(1500)),
                waitMs(1000),
                follow(follower,flowerToBack()),
                follow(follower,backToFlower()).raceWith(waitMs(1500)),
                waitMs(1000)
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(cycles, robot.aimAtHive(), robot.enableFlywheel()),
                follow(follower, park()),
                waitMs(3000),
                robot.intake.off()
        );
    }
}
