package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Auto", group = "Competition")
public class RedCycleGardenParkAuto extends OpMode {
    private Robot robot;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses (red side, blue is mirrored in start())
    // TODO: real poses
    private final Pose redStartPose = poseFactory.of(56, 8, 90);
    private final Pose redStartLaunchPose = poseFactory.of(56, 20.219975, 90);
    private final Pose redGardenBorderPose = poseFactory.of(28.4, 9.3, 180);
    private final Pose redGardenIntakePose = poseFactory.of(8.47877, 8.4, 185);
    private final Pose redBottomParkPose = poseFactory.of(12.2616, 91.7745, 90);

    private Pose startPose;
    private Pose launchPose;
    private Pose parkPose;

    // Path methods
    private Path startToLaunch() {
        return line(redStartPose, redStartLaunchPose).linear(redStartPose, redStartLaunchPose);
    }

    private Path prepareGardenIntake() {
        return line(redStartLaunchPose, redGardenBorderPose).linear(redStartLaunchPose, redGardenBorderPose);
    }

    private Path gardenIntake() {
        return line(redGardenBorderPose, redGardenIntakePose).linear(redGardenBorderPose, redGardenIntakePose);
    }

    private Path finishGardenIntake() {
        return line(redGardenIntakePose, redGardenBorderPose).linear(redGardenIntakePose, redGardenBorderPose);
    }

    private Path park() {
        return line(redGardenBorderPose, redBottomParkPose).linear(redGardenBorderPose, redBottomParkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToLaunch()),
                follow(follower, prepareGardenIntake()),
                follow(follower, gardenIntake()),
                follow(follower, finishGardenIntake()),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();
        follower.setPose(redStartPose);
        follower.update();
    }

    @Override
    public void start() {
        robot.start();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        robot.update(); // updates the follower
        Scheduler.execute();

        telemetry.addData("Start", redStartPose);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }
}
