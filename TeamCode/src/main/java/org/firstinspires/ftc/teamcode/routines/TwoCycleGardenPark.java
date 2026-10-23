package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * 2 Cycles (Garden) + Park (doc/auto-pathing.md, doc/autoPP/red_2cycle_park.pp): shoot the preloads, collect the
 * garden pollen, drive up the middle and shoot it from the flower-side launch spot, then park on the top side of the
 * loading zone.
 */
public class TwoCycleGardenPark extends AutoRoutine {
    public TwoCycleGardenPark(Robot robot, Alliance alliance) {
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

    private Path gardenIntake() {
        return line(poses.gardenBorder, poses.gardenIntake).linear(poses.gardenBorder, poses.gardenIntake);
    }

    private Path gardenToCenterLine() {
        return line(poses.gardenIntake, poses.gardenCenterLine).linear(poses.gardenIntake, poses.gardenCenterLine);
    }

    private Path centerLineToFlowerLaunch() {
        return line(poses.gardenCenterLine, poses.flowerLaunch).linear(poses.gardenCenterLine, poses.flowerLaunch);
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
                follow(follower, gardenIntake()),
                follow(follower, gardenToCenterLine()),
                robot.intake.off(),

                // Up the middle to the other side and shoot the garden pollen
                follow(follower, centerLineToFlowerLaunch()),
                robot.shoot(),
                // Second tip
                robot.flipHive()
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(cycles, robot.aimAtHive(), robot.enableFlywheel()),
                follow(follower, park())
        );
    }
}
