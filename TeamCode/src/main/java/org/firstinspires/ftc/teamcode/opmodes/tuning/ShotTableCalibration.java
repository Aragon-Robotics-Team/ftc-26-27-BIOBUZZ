package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Drive.SLOW_MODE_SCALE;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_LEFT_CORNER;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.RED_RIGHT_CORNER;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.ShotTable;
import org.firstinspires.ftc.teamcode.util.HoldButton;
import org.firstinspires.ftc.teamcode.util.PanelsDisplay;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Fill the shot table (core.ShotTable): the flywheel speed that scores from each distance. No model involved: you set
 * the speed by hand, and one button saves the robot's pose and that speed together.
 *
 * <p>Localize first (gamepad 2 share / options with the robot pushed into a corner, during init or after start), and
 * check the robot sits in the right place on Panels' Field tab. Hive vision picks the up cell like in TeleOp; the
 * turret aims at it. Then for each spot:
 * 1. Drive there and stop. Flywheel on (gamepad 1 RB). It starts at the table's speed for this distance if there is
 *    one (gamepad 2 left stick button jumps back to it), else the last speed you used.
 * 2. Shoot (hold gamepad 1 RT) and change the speed (gamepad 2 dpad up / down small steps, RB / LB big steps) until
 *    balls go through the middle of the opening.
 * 3. Gamepad 2 cross saves the distance and speed.
 * Gamepad 2 triangle undoes the last save; hold it to clear them all.
 *
 * <p>The table lives in code (core.ShotTable), so saving doesn't change what TeleOp and Auto use. Each save is listed
 * as a {@code {distance, speed},} line on Panels and in the robot log (tag ShotTable): paste them into
 * ShotTable.POINTS, pick one speed per spot where saves disagree, and redeploy. The saves are kept until the robot
 * app restarts, so stopping and re-running this OpMode doesn't lose them. Cover from the closest shot you'll take to
 * the farthest, every 6 to 12 in, and closer together where the speed changes fast.
 *
 * <p>Gamepad 1 drives like TeleOp (left stick field-centric, right stick x turns, hold LT slow), triangle / cross
 * toggle intake / outtake. Gamepad 2 dpad left / right nudge the turret, square / circle pick the LEFT / RIGHT cell.
 * Alliance at init: gamepad 1 X / B.
 */
@Configurable
@TeleOp(name = "Shot Table Calibration", group = "Tuning")
public class ShotTableCalibration extends OpMode {
    public static double smallStep = 10;             // ticks/sec per dpad press
    public static double bigStep = 50;               // ticks/sec per bumper press
    public static double startVelocity = 1600;       // ticks/sec when the table has nothing yet
    // Saving needs the robot standing still: the table is for standing shots
    public static double stillSpeed = 2;              // in/s
    public static double stillTurnDegPerSec = 5;
    // The flywheel has to be this close to the speed you set, so the save is what the balls actually saw
    public static double atSpeedTolerance = 40;       // ticks/sec

    // Static so the speed survives stopping and re-running the OpMode
    private static double velocity = Double.NaN;

    private Robot robot;
    private TelemetryManager panels;
    private PanelsDisplay panelsDisplay; // Field tab only
    /** One save: the distance and speed, and where the robot was, for the comment on its line. */
    private static final class Save {
        final double distance, velocity;
        final Pose pose;
        final String alliance, hive;

        Save(double distance, double velocity, Pose pose, String alliance, String hive) {
            this.distance = distance;
            this.velocity = velocity;
            this.pose = pose;
            this.alliance = alliance;
            this.hive = hive;
        }

        /** The line to paste into ShotTable.POINTS. */
        String line() {
            return String.format(Locale.US, "{%.1f, %.0f}, // %s %s, robot %.0f, %.0f, %.0f°", distance, velocity,
                    alliance, hive, pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
    }

    // Static so the saves survive stopping and re-running the OpMode; newest last
    private static final List<Save> saves = new ArrayList<>();

    private final HoldButton deleteButton = new HoldButton(1.5);
    private String message = "";
    /** Red-side corner picked during init, or null. Turned into a pose at start, once the alliance is final. */
    private Pose initCorner = null;

    @Override
    public void init() {
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        robot.hiveVision.setEnabled(true); // the up cell, like TeleOp
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsDisplay = new PanelsDisplay();
        if (MatchState.pose != null) {
            robot.drivebase.setPose(MatchState.pose);
            robot.launcher.setCurrentTurretAngle(MatchState.turretAngle);
        }
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, 1, telemetry);
        if (gamepad2.shareWasPressed()) initCorner = RED_LEFT_CORNER;
        if (gamepad2.optionsWasPressed()) initCorner = RED_RIGHT_CORNER;
        telemetry.addData("Shot table", ShotTable.size() + " points in code, " + saves.size() + " saves this session");
        telemetry.addData("Start pose", (initCorner == null ? "none"
                : initCorner == RED_LEFT_CORNER ? "LEFT corner" : "RIGHT corner")
                + "   (gamepad 2 share / options, now or after start)");
        telemetry.addLine("Turret must face forward at init");
        telemetry.update();
    }

    @Override
    public void start() {
        if (initCorner != null) robot.drivebase.setPose(MatchState.alliance.apply(initCorner));
        robot.start();
        double headingOffset = MatchState.alliance == Alliance.BLUE ? Math.PI : 0;
        schedule(
                robot.drivebase.driveFieldCentric(
                        () -> -gamepad1.left_stick_y * driveScale(),
                        () -> -gamepad1.left_stick_x * driveScale(),
                        () -> -gamepad1.right_stick_x * driveScale(),
                        headingOffset),
                // Standing shots: the turret points straight at the cell
                robot.launcher.aimAt(robot.drivebase::getPose, Velocity::zero, robot::hiveTarget)
        );
    }

    @Override
    public void loop() {
        bindControls();

        robot.update();
        Scheduler.execute();

        showTelemetry();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }

    private double driveScale() {
        return gamepad1.left_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
    }

    /** The table's speed for where the robot is now (the one in code), or NaN if it's empty. */
    private double tableVelocityHere() {
        return ShotTable.velocityAt(robot.distanceToHive());
    }

    private void bindControls() {
        // ---- Gamepad 1: same as TeleOp ----
        if (gamepad1.triangleWasPressed()) schedule(robot.intake.toggleIntake());
        if (gamepad1.crossWasPressed()) schedule(robot.intake.toggleOuttake());

        if (gamepad1.rightBumperWasPressed()) {
            double fromTable = tableVelocityHere();
            if (!Double.isNaN(fromTable)) velocity = Math.round(fromTable);
            else if (Double.isNaN(velocity)) velocity = startVelocity;
            schedule(robot.launcher.holdVelocity(() -> velocity));
        }
        if (gamepad1.leftBumperWasPressed()) schedule(robot.launcher.idle());

        if (gamepad1.rightTriggerWasPressed()) schedule(robot.startShooting());
        if (gamepad1.rightTriggerWasReleased()) schedule(robot.stopShooting());

        // ---- Gamepad 2: speed, aim, saving ----
        if (gamepad2.dpadUpWasPressed()) changeVelocity(+smallStep);
        if (gamepad2.dpadDownWasPressed()) changeVelocity(-smallStep);
        if (gamepad2.rightBumperWasPressed()) changeVelocity(+bigStep);
        if (gamepad2.leftBumperWasPressed()) changeVelocity(-bigStep);
        if (gamepad2.leftStickButtonWasPressed()) {
            double fromTable = tableVelocityHere();
            if (Double.isNaN(fromTable)) {
                message = "Table is empty: nothing to jump to";
            } else {
                velocity = Math.round(fromTable);
                message = String.format("Speed set to the table's %.0f", velocity);
            }
        }

        if (gamepad2.dpadLeftWasPressed()) schedule(robot.launcher.nudgeTurret(+1));
        if (gamepad2.dpadRightWasPressed()) schedule(robot.launcher.nudgeTurret(-1));

        if (gamepad2.squareWasPressed()) schedule(robot.setHive(Hive.LEFT));
        if (gamepad2.circleWasPressed()) schedule(robot.setHive(Hive.RIGHT));

        if (gamepad2.shareWasPressed()) schedule(robot.resetPose(RED_LEFT_CORNER));
        if (gamepad2.optionsWasPressed()) schedule(robot.resetPose(RED_RIGHT_CORNER));

        if (gamepad2.crossWasPressed()) save();
        switch (deleteButton.update(gamepad2.triangle, getRuntime())) {
            case TAP:
                if (saves.isEmpty()) {
                    message = "No saves to undo";
                } else {
                    Save removed = saves.remove(saves.size() - 1);
                    message = String.format("Undid %.1f in → %.0f ticks/s", removed.distance, removed.velocity);
                }
                break;
            case HOLD:
                saves.clear();
                message = "Cleared every save";
                break;
            default:
                break;
        }
    }

    private void changeVelocity(double by) {
        if (Double.isNaN(velocity)) velocity = startVelocity;
        velocity = Range.clip(velocity + by, 0, RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY);
    }

    /** Why this spot can't be saved right now, or null if it can. */
    private String cantSave() {
        if (!robot.launcher.isSpinning()) return "flywheel is off (gamepad 1 RB)";
        Velocity v = robot.drivebase.getVelocity();
        if (Math.hypot(v.vx, v.vy) > stillSpeed || Math.abs(v.omega) > Math.toRadians(stillTurnDegPerSec)) {
            return "robot is moving";
        }
        if (Math.abs(robot.launcher.getFlywheelVelocity() - robot.launcher.getTargetVelocity()) > atSpeedTolerance) {
            return "flywheel not at speed yet";
        }
        return null;
    }

    private void save() {
        String reason = cantSave();
        if (reason != null) {
            message = "Not saved: " + reason;
            return;
        }
        double distance = robot.distanceToHive();
        double ticks = robot.launcher.getTargetVelocity();
        Save save = new Save(distance, ticks, robot.drivebase.getPose(), MatchState.alliance.toString(),
                MatchState.hive.toString());
        saves.add(save);
        // The whole list each time, so the newest log line always has everything to copy
        StringBuilder all = new StringBuilder("Shot table saves, paste into ShotTable.POINTS:");
        for (Save s : sorted()) all.append('\n').append(s.line());
        RobotLog.ii("ShotTable", all.toString());
        message = String.format("Saved %.1f in → %.0f ticks/s (%d saves)", distance, ticks, saves.size());
    }

    /** The saves, nearest first. */
    private static List<Save> sorted() {
        List<Save> list = new ArrayList<>(saves);
        list.sort((a, b) -> Double.compare(a.distance, b.distance));
        return list;
    }

    private void showTelemetry() {
        Pose pose = robot.drivebase.getPose();
        double distance = robot.distanceToHive();
        double fromTable = ShotTable.velocityAt(distance);

        panels.addLine("== SHOT TABLE CALIBRATION ==");
        panels.addLine("G1: drive, RB/LB flywheel on/off, hold RT shoot. G2: dpad up/down ±" + (int) smallStep
                + ", RB/LB ±" + (int) bigStep + ", L-stick = table speed, cross save, triangle undo (hold: clear all)");
        panels.addData("Alliance / cell", MatchState.alliance + " / " + MatchState.hive + "   (G2 square / circle)");
        panels.addData("Pose", String.format("%.1f, %.1f, %.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        panels.addData("Distance (in)", String.format("%.1f", distance));
        panels.addData("Flywheel set / actual", String.format("%.0f / %.0f",
                robot.launcher.getTargetVelocity(), robot.launcher.getFlywheelVelocity()));
        panels.addData("Table says here", Double.isNaN(fromTable) ? "(empty)"
                : String.format("%.0f%s", fromTable, ShotTable.covers(distance) ? "" : " (out of range)"));
        String reason = cantSave();
        panels.addData("Save (G2 cross)", reason == null ? "ready" : reason);
        if (!message.isEmpty()) panels.addLine(message);

        double[][] points = new double[saves.size()][];
        for (int i = 0; i < points.length; i++) points[i] = new double[] {saves.get(i).distance, saves.get(i).velocity};
        double fromSaves = new ShotTable.Table(points).velocityAt(distance);
        if (!Double.isNaN(fromSaves)) panels.addData("Saves say here", String.format("%.0f", fromSaves));

        panels.addLine("");
        panels.addLine(String.format("== %d SAVES: paste into ShotTable.POINTS (core/ShotTable.java), also in the robot"
                + " log, tag ShotTable ==", saves.size()));
        for (Save s : sorted()) panels.addLine(s.line());
        panels.addLine(String.format("(The table in code has %d points; saves don't change it until pasted and"
                + " redeployed)", ShotTable.size()));
        panels.update(telemetry);
        panelsDisplay.drawField(robot);
    }
}
