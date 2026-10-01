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

/** Shoot the preloads, intake the garden balls and shoot them, then park in the loading zone. */
public class CycleGardenPark extends AutoRoutine {
    public CycleGardenPark(Robot robot, Alliance alliance) {
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

    private Path finishGardenIntake() {
        return line(poses.gardenIntake, poses.gardenBorder).linear(poses.gardenIntake, poses.gardenBorder);
    }

    private Path gardenToLaunch() {
        return line(poses.gardenBorder, poses.startLaunch).linear(poses.gardenBorder, poses.startLaunch);
    }

    private Path park() {
        return line(poses.startLaunch, poses.bottomPark).linear(poses.startLaunch, poses.bottomPark);
    }

    @Override
    public Command autoRoutine() {
        Command cycles = sequential(
                // Preloads
                follow(follower, startToLaunch()),
                robot.shoot(),
                // Tip
                robot.flipHive(),

                // Garden: intake on the way in, stop once back out
                robot.intake.intake(),
                follow(follower, prepareGardenIntake()),
                follow(follower, gardenIntake()),
                follow(follower, finishGardenIntake()),
                robot.intake.off(),

                follow(follower, gardenToLaunch()),
                robot.shoot()
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(cycles, robot.aimAtHive(), robot.enableFlywheel()),
//                robot.launcher.idle(),
                follow(follower, park())
        );
    }
}
