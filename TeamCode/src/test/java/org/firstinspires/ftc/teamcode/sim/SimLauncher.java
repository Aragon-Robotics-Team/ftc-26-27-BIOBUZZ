package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.PossessionState;
import org.horizon36596.simloop.field.TriggerVolume;
import org.horizon36596.simloop.viz.ShotArc;
import org.horizon36596.simloop.viz.ShotArcRecorder;
import org.psilynx.psikit.core.Logger;

import java.util.ArrayList;
import java.util.List;

/**
 * The robot's ball path in the sim, from what the code commands: the intake takes pollen (never nectar) that reaches its mouth while it
 * runs; with the gate open and the intake running, stored balls go to the flywheel one at a time and leave from the
 * turret at the hood angle, as fast as the flywheel is really spinning (FLYWHEEL_TICKS_PER_EXIT_SPEED), carrying the
 * robot's motion. SimLoop flies them with gravity and drag and no lift, like ShotSolver with LIFT_COEFFICIENT 0 (the
 * counter-rollers leave the ball without spin).
 *
 * Each shot is logged (Shots/...) as a pair of arcs: the one it flew, and the one that would have gone through the
 * middle of the up cell's opening from the same launch point and direction (the speed found by trial with the same
 * drag). They differ by the flywheel's speed error and any LIFT_COEFFICIENT ShotSolver is given; Shots/missInches says
 * by how much.
 */
final class SimLauncher {
    // PLACEHOLDERS: how many balls the robot holds, how fast the intake takes them and how fast stored balls feed
    static final int CAPACITY = 4;
    private static final double INTAKE_INTERVAL = 0.15; // s between balls the intake can take
    private static final double FEED_INTERVAL = 0.25;   // s between balls fed to the flywheel
    private static final int SHOT_SLOTS = 3;            // recent shots kept on screen

    private final GamePieceTracker field;
    private final FakeMotor intake, flywheel;
    private final FakeServo gate;
    // The intake's mouth: just inside the front bumper to past INTAKE_REACH, between the side plates (RobotConstants)
    private final TriggerVolume mouth;
    // The intake only takes pollen: nectar is too big for it, so the robot pushes nectar like any other obstacle
    private static final List<String> POLLEN_ONLY = java.util.Collections.singletonList(Ball.Type.POLLEN.name());
    private final ShotArcRecorder shots = new ShotArcRecorder(SHOT_SLOTS);

    private double intakeWait, feedWait;
    private int flying = -1;                       // id of the newest shot while it's in the air
    private final List<double[]> flown = new ArrayList<>();
    private int shotsFired;

    SimLauncher(GamePieceTracker field, FakeMotor intake, FakeServo gate, FakeMotor flywheel) {
        this.field = field;
        this.intake = intake;
        this.gate = gate;
        this.flywheel = flywheel;
        double front = RobotConstants.Field.ROBOT_LENGTH / 2, reach = RobotConstants.BallVision.INTAKE_REACH;
        double back = front - 1, out = reach + 1.5; // a ball against the bumper has its center 1.4 in past it
        mouth = new TriggerVolume("intake", (back + out) / 2, 0, out - back, RobotConstants.BallVision.INTAKE_WIDTH);
    }

    /** Balls the robot starts holding (the match's preload). */
    void preload(Ball.Type type, int count) {
        for (int i = 0; i < count; i++) field.addHeldPiece(type.name(), type.diameter / 2);
    }

    int shotsFired() {
        return shotsFired;
    }

    /**
     * One tick, after GamePieceTracker.updateRobotPose and before advance. Field frame, inches and radians:
     * the turret axis's position, its field heading, the robot's velocity, and the point a shot should go through.
     */
    void update(double deltaTime, double axisX, double axisY, double turretHeading, double vx, double vy,
                double[] aim) {
        boolean intakeRunning = intake.getPower() > 0.1;
        boolean gateOpen = gate.getPosition()
                > (RobotConstants.Gate.CLOSED_POSITION + RobotConstants.Gate.OPEN_POSITION) / 2;
        intakeWait -= deltaTime;
        feedWait -= deltaTime;

        boolean room = field.getHeldCount() < CAPACITY;
        if (intakeWait <= 0 && field.tryAcquire(mouth, intakeRunning && room, POLLEN_ONLY, 1) > 0) intakeWait = INTAKE_INTERVAL;

        int next = field.nextHeldPieceId();
        if (gateOpen && intakeRunning && feedWait <= 0 && next >= 0) {
            feedWait = FEED_INTERVAL;
            launch(next, axisX, axisY, turretHeading, vx, vy, aim);
        }
        followFlight();
    }

    private void launch(int id, double x, double y, double heading, double vx, double vy, double[] aim) {
        double hood = RobotConstants.Launcher.HOOD_ANGLE;
        double r = RobotConstants.Launcher.EXIT_RADIUS;
        double ex = x + r * Math.cos(heading), ey = y + r * Math.sin(heading);
        double ez = RobotConstants.Launcher.EXIT_HEIGHT;
        double speed = Math.abs(flywheel.getVelocity()) / RobotConstants.Launcher.FLYWHEEL_TICKS_PER_EXIT_SPEED;
        double horizontal = speed * Math.cos(hood);
        field.launch(id, "launcher", ex, ey, ez, horizontal * Math.cos(heading) + vx,
                horizontal * Math.sin(heading) + vy, speed * Math.sin(hood));
        shotsFired++;

        // The arc that would have scored from here: same launch point and direction, the speed that reaches the aim
        ShotArc.Launch ideal = new ShotArc.Launch(ex, ey, ez, speedToReach(ex, ey, ez, heading, hood, aim), heading,
                hood);
        shots.fire(ShotArc.ballistic(ideal, dragPerInch(), aim[2], 0.02), aim[0], aim[1], aim[2]);
        flying = id;
        flown.clear();
        flown.add(new double[] {ex, ey, ez});
    }

    /** Grows the newest shot's flown arc until it lands or scores. */
    private void followFlight() {
        if (flying < 0) return;
        GamePiece p = field.getPiece(flying);
        flown.add(new double[] {p.getFieldXInches(), p.getFieldYInches(), p.getFieldZInches()});
        double[] xyz = new double[3 * flown.size()];
        for (int i = 0; i < flown.size(); i++) System.arraycopy(flown.get(i), 0, xyz, 3 * i, 3);
        shots.setActual(ShotArc.ofPoints(xyz));
        if (p.getState() != PossessionState.IN_FLIGHT || p.isSleeping() || flown.size() > 150) flying = -1;
    }

    /** The exit speed whose arc (with drag) comes down through the aim point's height nearest it: bisection. */
    static double speedToReach(double x, double y, double z, double heading, double hood, double[] aim) {
        double target = Math.hypot(aim[0] - x, aim[1] - y);
        double lo = 50, hi = 600;
        for (int i = 0; i < 40; i++) {
            double mid = (lo + hi) / 2;
            ShotArc arc = ShotArc.ballistic(new ShotArc.Launch(x, y, z, mid, heading, hood), dragPerInch(), aim[2], 0.02);
            int last = arc.size() - 1;
            double reached = Math.hypot(arc.x(last) - x, arc.y(last) - y);
            if (arc.z(last) < aim[2] - 0.5 || reached < target) lo = mid; else hi = mid;
        }
        return (lo + hi) / 2;
    }

    private static double dragPerInch() {
        return SimField.dragPerInch();
    }

    void record() {
        shots.record("Shots");
        Logger.recordOutput("Sim/BallsHeld", field.getHeldCount());
        Logger.recordOutput("Sim/ShotsFired", shotsFired);
    }
}
