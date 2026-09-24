package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Auto", group = "Competition")
public class Auto extends OpMode {
    private enum State {
        DRIVE_TO_LAUNCH,
        LAUNCH,
        PARK,
        DONE
    }

    // TODO: real field poses (inches, radians). Written for red; Alliance.apply mirrors them for blue.
    private static final Pose START_POSE = Pose.zero();
    private static final Pose LAUNCH_POSE = Pose.zero();
    private static final Pose PARK_POSE = Pose.zero();

    private Pose startPose;
    private Pose launchPose;
    private Pose parkPose;

    private Robot robot;
    private State state;
    private final Timer stateTimer = new Timer();

    private Path toLaunch;
    private Path toPark;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void init_loop() {
        // TODO: start position selection
        MatchState.selectAlliance(gamepad1, telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        startPose = MatchState.alliance.apply(START_POSE);
        launchPose = MatchState.alliance.apply(LAUNCH_POSE);
        parkPose = MatchState.alliance.apply(PARK_POSE);
        robot.drivebase.setPose(startPose);
        buildPaths();

        robot.start();
        setState(State.DRIVE_TO_LAUNCH);
    }

    @Override
    public void loop() {
        updateStateMachine();

        robot.update();
        // Saved every loop so TeleOp gets the latest pose however Auto ends
        saveMatchState();
        telemetry.addData("Alliance", MatchState.alliance);
        telemetry.addData("State", state);
        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private void saveMatchState() {
        MatchState.pose = robot.drivebase.getPose();
        MatchState.turretAngle = robot.launcher.getTurretAngle();
    }

    private void buildPaths() {
        toLaunch = line(startPose, launchPose).linear(startPose, launchPose);
        toPark = line(launchPose, parkPose).linear(launchPose, parkPose);
    }

    private void updateStateMachine() {
        switch (state) {
            case DRIVE_TO_LAUNCH:
                // TODO
                break;
            case LAUNCH:
                // TODO
                break;
            case PARK:
                // TODO
                break;
            case DONE:
                break;
        }
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }
}
