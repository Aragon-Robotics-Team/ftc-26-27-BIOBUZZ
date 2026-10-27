package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayDeque;

/**
 * Checks the Pinpoint pod offsets: spin the robot in place and watch its position on the Panels field.
 * The position should stay on the start point. If it traces a circle, the pod offsets in pedro.Constants are off,
 * and the bigger the circle, the further off they are.
 *
 * The field shows the trail magnified ZOOM times around the field center, with rings at 1 in and 2 in of real drift.
 * The Graph shows x and y: a spin with wrong offsets makes them wave up and down once per turn.
 * Uses the localizer and drivetrain directly, so it runs before the Foresight Tuner.
 * Left stick drives, right stick x turns, A clears the trail and puts the robot back at 0, 0.
 */
@Configurable
@TeleOp(name = "Localization Test", group = "Tuning")
public class LocalizationTest extends OpMode {
    public static double ZOOM = 10;           // field inches per inch of drift
    public static double TRAIL_STEP = 0.05;   // inches the robot has to move before the trail gets a new point
    public static int TRAIL_POINTS = 600;

    private PinpointLocalizer localizer;
    private Mecanum drivetrain;
    private TelemetryManager panels;
    private FieldManager field;
    private final ArrayDeque<double[]> trail = new ArrayDeque<>();
    private double maxDrift;
    private double turns;
    private double lastHeading;

    @Override
    public void init() {
        localizer = Constants.localizer(hardwareMap);
        drivetrain = Constants.drivetrain(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());
        reset();
    }

    @Override
    public void start() {
        reset();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) reset();
        drivetrain.drive(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x), true);
        localizer.update();

        Pose pose = localizer.pose();
        double drift = Math.hypot(pose.x(), pose.y());
        maxDrift = Math.max(maxDrift, drift);
        double turned = pose.heading() - lastHeading;
        turned = Math.atan2(Math.sin(turned), Math.cos(turned)); // the short way, across ±180°
        turns += turned / (2 * Math.PI);
        lastHeading = pose.heading();

        double[] last = trail.peekLast();
        if (last == null || Math.hypot(pose.x() - last[0], pose.y() - last[1]) >= TRAIL_STEP) {
            trail.addLast(new double[]{pose.x(), pose.y()});
            if (trail.size() > TRAIL_POINTS) trail.removeFirst();
        }
        if (field.getShouldUpdateCanvas()) draw(pose);

        panels.addLine("Spin in place with the right stick. A: clear and reset to 0, 0");
        panels.addData("x (in)", pose.x());
        panels.addData("y (in)", pose.y());
        panels.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        panels.addData("Drift (in)", drift);
        panels.addData("Max drift (in)", maxDrift);
        panels.addData("Turns", turns);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        drivetrain.drive(new DrivePowers(0, 0, 0), true);
    }

    private void reset() {
        localizer.setPose(Pose.zero());
        trail.clear();
        maxDrift = 0;
        turns = 0;
        lastHeading = 0;
    }

    /** The trail and the robot, ZOOM times bigger, around the field center. */
    private void draw(Pose pose) {
        double cx = 72, cy = 72;
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), "gray", 0.3);
        for (double ring : new double[]{1, 2}) {
            field.moveCursor(cx, cy);
            field.circle(ring * ZOOM);
        }

        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), "orange", 0.5);
        double[] previous = null;
        for (double[] point : trail) {
            if (previous != null) {
                field.moveCursor(cx + previous[0] * ZOOM, cy + previous[1] * ZOOM);
                field.line(cx + point[0] * ZOOM, cy + point[1] * ZOOM);
            }
            previous = point;
        }

        // The robot: a dot where it is, and a line the way it faces
        double x = cx + pose.x() * ZOOM, y = cy + pose.y() * ZOOM;
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), PanelsField.INSTANCE.getBLUE(), 0.5);
        field.moveCursor(x, y);
        field.circle(1);
        field.line(x + 6 * Math.cos(pose.heading()), y + 6 * Math.sin(pose.heading()));
        field.update();
    }
}
