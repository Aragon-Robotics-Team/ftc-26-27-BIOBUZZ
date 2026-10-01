package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.routines.PlannedPath;
import org.firstinspires.ftc.teamcode.routines.Routines;
import org.firstinspires.ftc.teamcode.util.PathLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Drives one path copied from tools/path-planner.html and logs it, to try it on the real robot and to collect runs for
 * the planner's Logs tab. It lists the planned paths used by any routine (paste the path into a routine first).
 * During init pick the alliance (X / B) and the path (dpad up / down), then place the robot at the pose shown.
 * Start drives the path once.
 */
@Autonomous(name = "Planned Path Test", group = "Tuning")
public class PlannedPathTest extends OpMode {
    // Static so the last pick is remembered between runs
    private static String selected = null;

    private Robot robot;
    private Follower follower;
    private Alliance builtFor = null;
    private Map<String, PlannedPath.Built> paths;
    private List<String> names = new ArrayList<>();
    private PathLog pathLog;
    private PlannedPath.Built running;
    private final ElapsedTime timer = new ElapsedTime();
    private double finishedAt = Double.NaN;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();
    }

    /** Builds every routine so the planned paths they use get registered with PlannedPath. */
    private void buildPaths() {
        PlannedPath.clearBuilt();
        for (Routines routine : Routines.values()) {
            try {
                routine.create(robot, MatchState.alliance).autoRoutine();
            } catch (RuntimeException e) {
                // a routine that can't be built just contributes no paths
            }
        }
        paths = PlannedPath.built();
        names = new ArrayList<>(paths.keySet());
        builtFor = MatchState.alliance;
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        if (builtFor != MatchState.alliance) buildPaths();
        if (names.isEmpty()) {
            telemetry.addLine("No planned paths: copy one from tools/path-planner.html into a routine.");
            telemetry.update();
            return;
        }
        int index = Math.max(0, names.indexOf(selected));
        if (gamepad1.dpadDownWasPressed()) index++;
        if (gamepad1.dpadUpWasPressed()) index--;
        index = Math.floorMod(index, names.size());
        selected = names.get(index);
        Pose start = paths.get(selected).start;
        telemetry.addData("Path", "%s   (%d/%d, dpad ↑↓)", selected, index + 1, names.size());
        telemetry.addData("Place robot at", "x %.1f  y %.1f  h %.0f°", start.x(), start.y(), Math.toDegrees(start.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        if (paths == null || !paths.containsKey(selected)) return;
        running = paths.get(selected);
        follower.setPose(running.start);
        robot.start();
        pathLog = PathLog.start(hardwareMap, follower, "TEST_" + selected, MatchState.alliance);
        schedule(follow(follower, running.path));
        timer.reset();
    }

    @Override
    public void loop() {
        robot.update(); // updates the follower
        Scheduler.execute();
        if (running == null) return;
        if (Double.isNaN(finishedAt) && timer.seconds() > 0.1 && !follower.following()) finishedAt = timer.seconds();

        Pose pose = follower.pose();
        telemetry.addData("Path", selected);
        telemetry.addData("Took", Double.isNaN(finishedAt) ? "driving…" : String.format("%.2f s", finishedAt));
        telemetry.addData("Follower", "%s, segment %d", follower.mode(), follower.pathIndex());
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (pathLog != null) pathLog.close();
        robot.stop();
    }
}
