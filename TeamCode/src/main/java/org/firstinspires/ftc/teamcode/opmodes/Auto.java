package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Auto", group = "Competition")
public class Auto extends OpMode {
    private enum State {
        DRIVE_TO_LAUNCH,
        LAUNCH,
        PARK,
        DONE
    }

    // TODO: real field poses (inches, radians)
    private static final Pose START_POSE = Pose.zero();
    private static final Pose LAUNCH_POSE = Pose.zero();
    private static final Pose PARK_POSE = Pose.zero();

    private Robot robot;
    private State state;
    private final Timer stateTimer = new Timer();

    private Path toLaunch;
    private Path toPark;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.drivebase.setPose(START_POSE);
        buildPaths();
    }

    @Override
    public void init_loop() {
        // TODO: alliance / start position selection
    }

    @Override
    public void start() {
        setState(State.DRIVE_TO_LAUNCH);
    }

    @Override
    public void loop() {
        updateStateMachine();

        robot.update();
        telemetry.addData("State", state);
        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private void buildPaths() {
        toLaunch = line(START_POSE, LAUNCH_POSE).linear(START_POSE, LAUNCH_POSE);
        toPark = line(LAUNCH_POSE, PARK_POSE).linear(LAUNCH_POSE, PARK_POSE);
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
