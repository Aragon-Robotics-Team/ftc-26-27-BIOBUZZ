package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.ROBOT_WIDTH;

import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathSegment;

import org.firstinspires.ftc.teamcode.core.ShotSolver;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * Panels view for the competition OpModes: numbers for the Graph tab, and on the Field tab the robot, the path it's
 * following, the hive cell it aims at and where the turret points. Only Panels gets these; the Driver Station keeps the OpMode's own telemetry.
 */
public class PanelsDisplay {
    private static final int PATH_SAMPLES = 20; // points per path segment

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final FieldManager field = PanelsField.INSTANCE.getField();

    public PanelsDisplay() {
        field.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());
    }

    /** Call once per loop. */
    public void update(Robot robot, double loopMs) {
        Pose pose = robot.drivebase.getPose();
        panels.addData("Flywheel target (ticks/s)", robot.launcher.getTargetVelocity());
        panels.addData("Flywheel actual (ticks/s)", robot.launcher.getFlywheelVelocity());
        panels.addData("Flywheel power", robot.launcher.getFlywheelPower());
        panels.addData("Turret target (deg)", Math.toDegrees(robot.launcher.getTurretTargetAngle()));
        panels.addData("Turret actual (deg)", Math.toDegrees(robot.launcher.getTurretAngle()));
        panels.addData("Loop (ms)", loopMs);
        panels.addData("x (in)", pose.x());
        panels.addData("y (in)", pose.y());
        panels.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        panels.update();

        drawField(robot);
    }

    /** Only the Field tab, for OpModes that send their own Panels telemetry (e.g. Shot Calibration). Once per loop. */
    public void drawField(Robot robot) {
        if (field.getShouldUpdateCanvas()) {
            draw(robot.drivebase.getFollower(), robot.drivebase.getPose(), robot.launcher, robot.hiveTarget());
        }
    }

    private void draw(Follower follower, Pose pose, Launcher launcher, Pose target) {
        // The path being followed: Auto's routine, or an auto-align path in TeleOp
        if (follower.following()) {
            field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), "green", 0.75);
            for (PathSegment segment : follower.currentPath().getSegments()) {
                Vector2D previous = segment.curve.get(0);
                for (int i = 1; i <= PATH_SAMPLES; i++) {
                    Vector2D point = segment.curve.get((double) i / PATH_SAMPLES);
                    field.moveCursor(previous.x(), previous.y());
                    field.line(point.x(), point.y());
                    previous = point;
                }
            }
        }

        // The robot: a circle about its size, and a line the way it faces
        double radius = Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2;
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), PanelsField.INSTANCE.getBLUE(), 0.75);
        field.moveCursor(pose.x(), pose.y());
        field.circle(radius);
        field.line(pose.x() + radius * Math.cos(pose.heading()), pose.y() + radius * Math.sin(pose.heading()));

        // Aim: the hive cell, and from the turret pivot as far as the cell, where the turret should point (gray) and
        // where it points (orange). Orange on gray through the cell = aimed; both off the cell = the pose is wrong.
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), "gold", 0.75);
        field.moveCursor(target.x(), target.y());
        field.circle(3);
        Pose pivot = ShotSolver.turretPivot(pose);
        double reach = Math.hypot(target.x() - pivot.x(), target.y() - pivot.y());
        aimLine(pivot, reach, pose.heading() + launcher.getTurretTargetAngle(), "gray");
        aimLine(pivot, reach, pose.heading() + launcher.getTurretAngle(), "orange");
        field.update();
    }

    private void aimLine(Pose from, double length, double fieldAngle, String color) {
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), color, 0.75);
        field.moveCursor(from.x(), from.y());
        field.line(from.x() + length * Math.cos(fieldAngle), from.y() + length * Math.sin(fieldAngle));
    }
}
