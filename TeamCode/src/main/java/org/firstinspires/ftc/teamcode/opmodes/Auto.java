package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.util.Html.bold;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.routines.AutoRoutine;
import org.firstinspires.ftc.teamcode.routines.FieldPoses;
import org.firstinspires.ftc.teamcode.routines.Routines;
import org.firstinspires.ftc.teamcode.util.Html;
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.firstinspires.ftc.teamcode.util.PanelsDisplay;
import org.firstinspires.ftc.teamcode.util.PathLog;

import java.util.List;

/** The only competition auto. Pick the alliance and routine during init; the routines live in {@link Routines}. */
@Autonomous(name = "Auto", group = "Competition")
public class Auto extends OpMode {
    // Static so the last pick is remembered between runs
    private static Routines selected = null;
    // Lined up on the routine's INIT spot and dragged to the start (else placed at the start pose). Gamepad 1 cross
    private static boolean fromInitSpot = true;
    // The Pinpoint settles (and calibrates its gyro) after INIT: the INIT spot is read this long after
    private static final double INIT_SPOT_READ_S = 0.5;

    private Robot robot;
    private Follower follower;
    private List<Routines> routines;
    private PathLog pathLog;
    private AutoRoutine running; // the one start() scheduled
    private PanelsDisplay panelsDisplay;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime sinceInit = new ElapsedTime();
    private Pose initSpotRaw = null; // the Pinpoint's pose with the robot on the INIT spot, null until read

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        Scheduler.reset();
        MatchLog.start("Auto");

        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();
        panelsDisplay = new PanelsDisplay();

        // New match: both alliances start with the RIGHT cell up
        MatchState.hive = Hive.RIGHT;

        routines = Routines.enabled();
        if (routines.isEmpty()) throw new IllegalStateException("Every routine is @Disabled");
        if (!routines.contains(selected)) selected = routines.get(0);
        sinceInit.reset();
    }

    @Override
    public void init_loop() {
        telemetry.addLine(Html.color(bold("Did you remember to zero the turret? :-)"), Html.YELLOW));
        MatchState.selectAlliance(gamepad1, 1, telemetry);
        int index = routines.indexOf(selected);
        if (gamepad1.dpadDownWasPressed()) index = (index + 1) % routines.size();
        if (gamepad1.dpadUpWasPressed()) index = (index + routines.size() - 1) % routines.size();
        selected = routines.get(index);
        AutoRoutine routine = selected.create(robot, MatchState.alliance);
        if (routine.usesBallVision()) {
            robot.ballVision.enable(false); // the camera takes a moment to open
            robot.ballVision.prepare();
        } else robot.ballVision.stop(); // opened scrolling past one that does: its pipeline would eat loop time

        telemetry.addData("Routine", "%s   (%d/%d, dpad ↑↓)", bold(selected.toString()), index + 1, routines.size());
        showStart(routine);
        // The turret was zeroed at INIT: if it's been turned since, the aim and the fused heading are off by as much
        double turret = Math.toDegrees(robot.launcher.readTurretAngle());
        if (Math.abs(turret) > 2) {
            telemetry.addLine(Html.color(bold(String.format("Turret reads %.0f°: face it forward and INIT again", turret)), Html.RED));
        } else telemetry.addData("Turret", "%.0f°", turret);
        // The pose leans on the hive's tags (and Solo Carry on them for which cell is up): check the Limelight sees them
        int tags = robot.hiveVision.tagsInView();
        if (tags > 0) telemetry.addData("Limelight", "%d hive tags in view", tags);
        else if (tags < 0) telemetry.addLine(Html.color(bold("No Limelight in the configuration"), Html.YELLOW));
        else if (robot.hiveVision.tagsWithout3d() > 0) {
            telemetry.addLine(Html.color(bold("Limelight sees hive tags but no 3D pose: turn on Full 3D in its pipeline"), Html.RED));
        } else telemetry.addLine(Html.color(bold("Limelight sees no hive tags (fine if the hive is out of its view;"
                + " don't turn the turret to check)"), Html.YELLOW));
        telemetry.update();
    }

    @Override
    public void start() {
        // Alliance and routine are picked in init_loop, so the routine is built here instead of init()
        AutoRoutine routine = selected.create(robot, MatchState.alliance);
        follower.setPose(startPose(routine));
        routine.setUp();

        robot.start();
        // Every run is logged for the path planner (tools/path-planner, Logs tab)
        pathLog = PathLog.start(hardwareMap, follower, selected.toString(), MatchState.alliance);
        schedule(routine.autoRoutine());
        running = routine;
        loopTimer.reset();
    }

    /** The INIT spot (read once the Pinpoint has settled, or again on triangle), the drag so far, and where that puts us. */
    private void showStart(AutoRoutine routine) {
        FusedLocalizer localizer = robot.drivebase.getLocalizer();
        localizer.update(); // only reads the Pinpoint: nothing moves
        Pose raw = localizer.getOdometryPose();
        if (initSpotRaw == null && sinceInit.seconds() > INIT_SPOT_READ_S) initSpotRaw = raw;
        if (gamepad1.triangleWasPressed()) initSpotRaw = raw;
        if (gamepad1.crossWasPressed()) fromInitSpot = !fromInitSpot;

        Pose start = routine.startPose();
        FieldPoses.InitSpot spot = routine.initSpot();
        if (spot == null || !fromInitSpot) {
            telemetry.addData("Place robot at", "x %.1f  y %.1f  h %.0f°", start.x(), start.y(), Math.toDegrees(start.heading()));
            if (spot != null) telemetry.addLine("(cross: line up on the INIT spot and drag it there instead)");
            return;
        }
        telemetry.addLine(bold("INIT spot: ") + spot.hint + "   (cross: placed at the start instead)");
        if (initSpotRaw == null) {
            telemetry.addLine(Html.color(bold("Hold still: reading the INIT spot"), Html.YELLOW));
            return;
        }
        Pose now = spot.afterDrag(initSpotRaw, raw);
        double dragged = Math.hypot(raw.x() - initSpotRaw.x(), raw.y() - initSpotRaw.y());
        double turned = Math.toDegrees(Math.abs(Math.atan2(Math.sin(now.heading() - spot.pose.heading()),
                Math.cos(now.heading() - spot.pose.heading()))));
        telemetry.addData("Dragged", "%.1f in, turned %.1f°  ->  start x %.1f  y %.1f  h %.0f°   (triangle: it's on the INIT spot now)",
                dragged, turned, now.x(), now.y(), Math.toDegrees(now.heading()));
        double off = Math.hypot(now.x() - start.x(), now.y() - start.y());
        if (dragged < 3) {
            telemetry.addLine(Html.color(bold("Not dragged yet: drag it to the wall"), Html.YELLOW));
        } else if (off > 6) {
            telemetry.addLine(Html.color(bold(String.format("%.0f in from where the route starts: check the INIT spot", off)), Html.RED));
        }
    }

    /** The routine's start pose, or with an INIT spot, that spot plus the drag the Pinpoint measured since. */
    private Pose startPose(AutoRoutine routine) {
        FieldPoses.InitSpot spot = routine.initSpot();
        if (spot == null || !fromInitSpot || initSpotRaw == null) return routine.startPose();
        FusedLocalizer localizer = robot.drivebase.getLocalizer();
        localizer.update();
        return spot.afterDrag(initSpotRaw, localizer.getOdometryPose());
    }

    @Override
    public void loop() {
        MatchLog.beginLoop();
        robot.update(); // updates the follower
        Scheduler.execute();

        // Saved every loop so TeleOp gets the latest pose however Auto ends
        MatchState.pose = follower.pose();
        MatchState.turretAngle = robot.launcher.getTurretAngle();

        Pose pose = follower.pose();
        telemetry.addLine(bold(selected.toString()) + Html.GAP + Html.alliance(MatchState.alliance)
                + Html.GAP + bold("Hive: " + MatchState.hive));
        telemetry.addLine(String.format("Pose  x %.1f  y %.1f  h %.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addLine("Follower " + follower.mode());
        if (running.status() != null) telemetry.addLine("Doing  " + running.status());
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        telemetry.addLine(String.format("Loop  %.1f ms  (%.0f Hz)", loopMs, 1000 / loopMs));

        telemetry.addLine(Html.color("──── details ────", Html.GRAY));
        robot.telemetry(telemetry);
        telemetry.update();

        panelsDisplay.update(robot, loopMs);
        robot.log(loopMs);
        MatchLog.endLoop();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (pathLog != null) pathLog.close();
        robot.stop();
        MatchLog.stop();
    }
}
