package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * 2 Cycles (Flower) + Park (doc/auto-pathing.md, doc/autoPP/red_2cycle_flower_park.pp): shoot the preloads, drive up
 * the middle to the rear-wall flower and collect its pollen, back off and shoot it, then park on the top side of the
 * loading zone.
 */
public class TwoCycleFlowerPark extends AutoRoutine {
    public TwoCycleFlowerPark(Robot robot, Alliance alliance) {
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

    private Path launchToCenterLine() {
        return line(poses.startLaunch, poses.gardenCenterLine).tangent();
    }

    private Path centerLineToFlowerLaunch() {
        return line(poses.gardenCenterLine, poses.flowerLaunch).linear(poses.gardenCenterLine, poses.flowerLaunch);
    }

    private Path flowerIntake() {
        return line(poses.flowerLaunch, poses.flowerIntake).linear(poses.flowerLaunch, poses.flowerIntake);
    }

    private Path flowerToShoot() {
        return line(poses.flowerIntake, poses.flowerShoot).linear(poses.flowerIntake, poses.flowerShoot);
    }

    private Path park() {
        return line(poses.flowerShoot, poses.topPark).linear(poses.flowerShoot, poses.topPark);
    }

    @Override
    public Command autoRoutine() {
        Command cycles = sequential(
                // Preloads
                follow(follower, startToLaunch()),
                robot.shoot(),
                // Tip
                robot.flipHive(),

                // Up the middle to the flower
                follow(follower, launchToCenterLine()),
                follow(follower, centerLineToFlowerLaunch()),

                // Flower: intake on the way in, give it a moment, then stop
                robot.intake.intake(),
                follow(follower, flowerIntake()),
                waitMs(RobotConstants.Intake.FLOWER_INTAKE_MS),
                robot.intake.off(),

                // Back off the flower and shoot its pollen
                follow(follower, flowerToShoot()),
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
