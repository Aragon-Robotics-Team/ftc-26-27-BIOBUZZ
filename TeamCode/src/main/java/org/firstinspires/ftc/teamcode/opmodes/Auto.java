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
import org.firstinspires.ftc.teamcode.Hive;
import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.routines.AutoRoutine;
import org.firstinspires.ftc.teamcode.routines.Routines;
import org.firstinspires.ftc.teamcode.util.Html;

import java.util.List;

/** The only competition auto. Pick the alliance and routine during init; the routines live in {@link Routines}. */
@Autonomous(name = "Auto", group = "Competition")
public class Auto extends OpMode {
    // Static so the last pick is remembered between runs
    private static Routines selected = null;

    private Robot robot;
    private Follower follower;
    private List<Routines> routines;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();

        // New match: both alliances start with the RIGHT cell up
        MatchState.hive = Hive.RIGHT;

        routines = Routines.enabled();
        if (routines.isEmpty()) throw new IllegalStateException("Every routine is @Disabled");
        if (!routines.contains(selected)) selected = routines.get(0);
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        int index = routines.indexOf(selected);
        if (gamepad1.dpadDownWasPressed()) index = (index + 1) % routines.size();
        if (gamepad1.dpadUpWasPressed()) index = (index + routines.size() - 1) % routines.size();
        selected = routines.get(index);

        telemetry.addData("Routine", "%s   (%d/%d, dpad ↑↓)", bold(selected.toString()), index + 1, routines.size());
        Pose start = selected.create(robot, MatchState.alliance).startPose();
        telemetry.addData("Place robot at", "x %.1f  y %.1f  h %.0f°", start.x(), start.y(), Math.toDegrees(start.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Alliance and routine are picked in init_loop, so the routine is built here instead of init()
        AutoRoutine routine = selected.create(robot, MatchState.alliance);
        follower.setPose(routine.startPose());

        robot.start();
        schedule(routine.autoRoutine());
        loopTimer.reset();
    }

    @Override
    public void loop() {
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
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        telemetry.addLine(String.format("Loop  %.1f ms  (%.0f Hz)", loopMs, 1000 / loopMs));

        telemetry.addLine(Html.color("──── details ────", Html.GRAY));
        robot.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        robot.stop();
    }
}
