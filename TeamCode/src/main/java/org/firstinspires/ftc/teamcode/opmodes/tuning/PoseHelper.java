package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Robot;

import java.util.ArrayList;
import java.util.List;

/**
 * Push the robot around by hand and record poses from odometry.
 *   B  save the current pose
 *   X  zero the pose (0, 0, 0°), keep saved poses
 *   A  zero the pose and clear saved poses
 */
@Configurable
@TeleOp(name = "Pose Helper", group = "Tuning")
public class PoseHelper extends OpMode {

    private Robot robot;
    private TelemetryManager panels;
    private final List<Pose> poseList = new ArrayList<>();
    private boolean lastA = false, lastB = false, lastX = false;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        reset();
    }

    @Override
    public void init_loop() {
        robot.update();
        updateTelemetry();
    }

    @Override
    public void start() {
        reset();
    }

    @Override
    public void loop() {
        robot.update(); // updates the follower / odometry

        boolean b = gamepad1.b;
        if (b && !lastB) poseList.add(robot.drivebase.getPose());
        lastB = b;

        boolean x = gamepad1.x;
        if (x && !lastX) zeroPose();
        lastX = x;

        boolean a = gamepad1.a;
        if (a && !lastA) reset();
        lastA = a;

        updateTelemetry();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }

    private void zeroPose() {
        robot.drivebase.setPose(new Pose(0, 0, 0));
    }

    private void reset() {
        zeroPose();
        poseList.clear();
    }

    private static String format(Pose p) {
        return String.format("x %.2f, y %.2f, heading %.1f°", p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    private void updateTelemetry() {
        panels.addLine("Current: " + format(robot.drivebase.getPose()));
        panels.addLine("");
        for (int i = 0; i < poseList.size(); i++) {
            panels.addLine("Pose " + (i + 1) + ": " + format(poseList.get(i)));
        }
        panels.update(telemetry);
    }
}