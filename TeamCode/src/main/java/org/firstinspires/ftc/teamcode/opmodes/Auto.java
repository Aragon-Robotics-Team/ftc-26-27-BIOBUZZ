package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.routines.AutoRoutine;
import org.firstinspires.ftc.teamcode.routines.Routines;

/** The only competition auto. Pick the alliance and routine during init; the routines live in {@link Routines}. */
@Autonomous(name = "Auto", group = "Competition")
public class Auto extends OpMode {
    // Static so the last pick is remembered between runs
    private static Routines selected = Routines.values()[0];

    private Robot robot;
    private Follower follower;

    @Override
    public void init() {
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        follower = robot.drivebase.getFollower();
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        if (gamepad1.dpadDownWasPressed()) selected = selected.next();
        if (gamepad1.dpadUpWasPressed()) selected = selected.previous();
        telemetry.addData("Routine", "%s   (%d/%d, dpad ↑↓)", selected, selected.ordinal() + 1, Routines.values().length);
        telemetry.addData("Place robot at", selected.create(robot, MatchState.alliance).startPose());
        telemetry.update();
    }

    @Override
    public void start() {
        // Alliance and routine are picked in init_loop, so the routine is built here instead of init()
        AutoRoutine routine = selected.create(robot, MatchState.alliance);
        follower.setPose(routine.startPose());

        robot.start();
        schedule(routine.autoRoutine());
    }

    @Override
    public void loop() {
        robot.update(); // updates the follower
        Scheduler.execute();

        // Saved every loop so TeleOp gets the latest pose however Auto ends
        MatchState.pose = follower.pose();
        MatchState.turretAngle = robot.launcher.getTurretAngle();

        telemetry.addData("Alliance", MatchState.alliance);
        telemetry.addData("Routine", selected);
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
