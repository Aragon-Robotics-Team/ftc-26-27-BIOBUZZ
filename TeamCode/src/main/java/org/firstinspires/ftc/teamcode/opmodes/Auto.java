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
public class Auto extends OpMode {
    private Robot robot;
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses (red side, blue is mirrored in start())
    // TODO: real poses
    private final Pose redLaunchPose = poseFactory.of(0, 0, 0);
    private final Pose redParkPose = poseFactory.of(0, 0, 0);

    private Pose startPose;
    private Pose launchPose;
    private Pose parkPose;

    // Path methods
    private Path startToLaunch() {
        return line(startPose, launchPose).linear(startPose, launchPose);
    }

    private Path park() {
        return line(launchPose, parkPose).linear(launchPose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToLaunch()),
                // TODO: launch
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        MatchState.selectStartPosition(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        // Alliance and start position are picked in init_loop, so poses are set here instead of init()
        startPose = MatchState.startPosition.pose(MatchState.alliance);
        launchPose = MatchState.alliance.apply(redLaunchPose);
        parkPose = MatchState.alliance.apply(redParkPose);
        follower.setPose(startPose);

        robot.start();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        robot.update(); // updates the follower
        Scheduler.execute();

        // Saved every loop so TeleOp gets the latest pose however Auto ends
        MatchState.pose = follower.pose();
        MatchState.turretAngle = robot.launcher.getTurretAngle();

        telemetry.addData("Alliance", MatchState.alliance);
        telemetry.addData("Start", MatchState.startPosition);
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
