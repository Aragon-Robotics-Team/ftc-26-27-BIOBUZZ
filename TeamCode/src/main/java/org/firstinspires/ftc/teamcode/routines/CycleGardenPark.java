package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

/** Drive to the launch spot, cycle through the garden, then park. */
public class CycleGardenPark extends AutoRoutine {
    // Poses (red side, rotated 180° for blue)
    private final Pose startPose = poseFactory.of(56, 8, 90);
    private final Pose startLaunchPose = poseFactory.of(56, 20.219975, 90);
    private final Pose gardenBorderPose = poseFactory.of(28.4, 9.3, 180);
    private final Pose gardenIntakePose = poseFactory.of(8.47877, 8.4, 185);
    private final Pose bottomParkPose = poseFactory.of(12.2616, 91.7745, 90);

    public CycleGardenPark(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }

    @Override
    public Pose startPose() {
        return startPose;
    }

    // Path methods
    private Path startToLaunch() {
        return line(startPose, startLaunchPose).linear(startPose, startLaunchPose);
    }

    private Path prepareGardenIntake() {
        return line(startLaunchPose, gardenBorderPose).linear(startLaunchPose, gardenBorderPose);
    }

    private Path gardenIntake() {
        return line(gardenBorderPose, gardenIntakePose).linear(gardenBorderPose, gardenIntakePose);
    }

    private Path finishGardenIntake() {
        return line(gardenIntakePose, gardenBorderPose).linear(gardenIntakePose, gardenBorderPose);
    }

    private Path park() {
        return line(gardenBorderPose, bottomParkPose).linear(gardenBorderPose, bottomParkPose);
    }

    @Override
    public Command autoRoutine() {
        return sequential(
                follow(follower, startToLaunch()),
                follow(follower, prepareGardenIntake()),
                follow(follower, gardenIntake()),
                follow(follower, finishGardenIntake()),
                follow(follower, park())
        );
    }
}
