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
 * runs, and the bottom pollen of a flower it's pushed up to (takeFromFlowers); with the gate open and the intake running, stored balls go to the flywheel one at a time and leave from the
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
    // The intake's mouth: at INTAKE_REACH, between the side plates (RobotConstants)
    private final TriggerVolume mouth;
    private static final double MOUTH_DEPTH = 0.5; // inches inside INTAKE_REACH a ball still counts as touching it
    // The intake only takes pollen: nectar is too big for it, so the robot pushes nectar like any other obstacle
    private static final List<String> POLLEN_ONLY = java.util.Collections.singletonList(Ball.Type.POLLEN.name());
    // A flower's bottom pollen comes out through its Retrieval Opening (G418.B: only pollen, only from the bottom),
    // as BiobuzzSim lets it: the robot on the field side, within 40° of straight out from the wall, and the pollen
    // between the side plates and within FLOWER_REACH of the mouth. Square against the column, the mouth is 3.65 in
    // from its axis (the column's 2.65 in and the mouth 1 in inside the bumper), so the rollers have to reach about
    // 2.25 in past the mouth, under the middle ring, to the ball. PLACEHOLDER: check that the real intake does
    private static final double FLOWER_REACH = 4.0;          // inches, mouth to the bottom pollen's center
    private static final double FLOWER_APPROACH_COS = Math.cos(Math.toRadians(40));
    private final ShotArcRecorder shots = new ShotArcRecorder(SHOT_SLOTS);

    private double intakeWait, feedWait;
    private java.util.Random spread;              // null: every shot exactly as the flywheel and turret send it
    private double speedSpread, headingSpread, speedBias;
    // load(): a constant drag on the flywheel (a battery below 12 V) and a kick for one tick as each ball goes
    // through, fractions of stall torque
    private double baseLoad, ballLoad;
    private boolean kicked;
    private int flying = -1;                       // id of the newest shot while it's in the air
    private final List<double[]> flown = new ArrayList<>();
    private int shotsFired;

    SimLauncher(GamePieceTracker field, FakeMotor intake, FakeServo gate, FakeMotor flywheel) {
        this.field = field;
        this.intake = intake;
        this.gate = gate;
        this.flywheel = flywheel;
        // A pollen is taken once it touches the mouth: SimLoop's trigger counts any ball that overlaps the box, so the
        // box is the half inch just inside the mouth (INTAKE_REACH), between the side plates with room for the ball.
        // That takes a ball whose center is within its radius in front of the mouth or up to MOUTH_DEPTH + r inside
        double r = Ball.Type.POLLEN.diameter / 2;
        double reach = RobotConstants.BallVision.INTAKE_REACH;
        double halfWidth = RobotConstants.BallVision.INTAKE_WIDTH / 2 - 2 * r;
        mouth = new TriggerVolume("intake", reach - MOUTH_DEPTH / 2, 0, MOUTH_DEPTH, 2 * halfWidth);
    }

    /** Balls the robot starts holding (the match's preload). */
    void preload(Ball.Type type, int count) {
        for (int i = 0; i < count; i++) field.addHeldPiece(type.name(), type.diameter / 2);
    }

    /** Each shot's exit speed off by this fraction and its direction by this many radians (standard deviations). */
    void spread(long seed, double speedFraction, double headingRadians) {
        spread = new java.util.Random(seed);
        speedSpread = speedFraction;
        headingSpread = headingRadians;
    }

    /**
     * The flywheel loaded as a real one is: baseLoad of stall torque all the time (the motor model has no voltage,
     * so a battery at V is a load of 1 - V/12 at full power), and ballLoad more for one tick as a ball goes through
     * (1 takes about 5% off its speed).
     */
    void load(double baseLoad, double ballLoad) {
        this.baseLoad = baseLoad;
        this.ballLoad = ballLoad;
    }

    /** Every shot's exit speed off by this fraction: a flywheel calibration or drag the solver doesn't know about. */
    void bias(double speedFraction) {
        speedBias = speedFraction;
    }

    int shotsFired() {
        return shotsFired;
    }

    /**
     * One tick, after GamePieceTracker.updateRobotPose and before advance. Field frame, inches and radians: the
     * robot's pose, the turret axis's position, its field heading, the robot's velocity, and the point a shot should
     * go through.
     */
    void update(double deltaTime, double robotX, double robotY, double robotHeading, double axisX, double axisY,
                double turretHeading, double vx, double vy, double[] aim) {
        boolean intakeRunning = intake.getPower() > 0.1;
        boolean gateOpen = gate.getPosition()
                > (RobotConstants.Gate.CLOSED_POSITION + RobotConstants.Gate.OPEN_POSITION) / 2;
        intakeWait -= deltaTime;
        feedWait -= deltaTime;
        if (baseLoad > 0 || ballLoad > 0) {
            double load = Math.min(1, baseLoad + (kicked ? ballLoad : 0));
            kicked = false;
            flywheel.setExternalTorqueFraction(-Math.signum(flywheel.getPhysicalVelocityTicksPerSecond()) * load);
        }

        boolean room = field.getHeldCount() < CAPACITY;
        if (intakeWait <= 0 && intakeRunning && room) {
            if (field.tryAcquire(mouth, true, POLLEN_ONLY, 1) > 0
                    || takeFromFlowers(robotX, robotY, robotHeading)) intakeWait = INTAKE_INTERVAL;
        }

        int next = field.nextHeldPieceId();
        if (gateOpen && intakeRunning && feedWait <= 0 && next >= 0) {
            feedWait = FEED_INTERVAL;
            launch(next, axisX, axisY, turretHeading, vx, vy, aim);
        }
        followFlight();
    }

    /**
     * Takes the bottom pollen of a flower the intake is up against (FLOWER_REACH), and wakes the ones above it so they
     * drop into its place. Nothing comes out when the bottom ball is nectar: it blocks the opening. Field frame.
     */
    private boolean takeFromFlowers(double robotX, double robotY, double heading) {
        double fx = Math.cos(heading), fy = Math.sin(heading);
        double reach = RobotConstants.BallVision.INTAKE_REACH;
        for (int i = 0; i < RobotConstants.Field.FLOWERS.length; i++) {
            double[] c = SimField.flowerCenter(i), in = SimField.flowerInward(i);
            double ox = robotX - c[0], oy = robotY - c[1];
            if ((ox * in[0] + oy * in[1]) < FLOWER_APPROACH_COS * Math.hypot(ox, oy)) continue; // not from the field side
            // The lowest ball in the column. The ones stacked on it are IN_FLIGHT to SimLoop (off the floor), asleep
            GamePiece bottom = null;
            for (GamePiece p : field.getPieces()) {
                if (!inColumn(p)) continue;
                if (Math.hypot(p.getFieldXInches() - c[0], p.getFieldYInches() - c[1]) > SimField.FLOWER_INNER_RADIUS) continue;
                if (p.getFieldZInches() > SimField.FLOWER_TUBE_BOTTOM + 2) continue; // well up the tube
                if (bottom == null || p.getFieldZInches() < bottom.getFieldZInches()) bottom = p;
            }
            // Only one resting in the lower ring: its top under the middle ring (BiobuzzSim's test), not one still falling
            if (bottom == null || bottom.getState() != PossessionState.LOOSE
                    || !bottom.getKind().equals(Ball.Type.POLLEN.name())
                    || bottom.getFieldZInches() + 0.6 * bottom.getRadiusInches() > SimField.FLOWER_TUBE_BOTTOM) continue;
            double dx = bottom.getFieldXInches() - robotX, dy = bottom.getFieldYInches() - robotY;
            double ahead = dx * fx + dy * fy, side = -dx * fy + dy * fx;
            if (ahead < reach || ahead > reach + FLOWER_REACH) continue;
            if (Math.abs(side) > RobotConstants.BallVision.INTAKE_WIDTH / 2 - bottom.getRadiusInches()) continue;
            // tryAcquire takes the lowest-id loose ball on the spot, judged on the floor only: if another loose one
            // is there too (one that fell in beside it), leave both rather than take the wrong one
            for (GamePiece p : field.getPieces()) {
                if (p.getId() < bottom.getId() && p.getState() == PossessionState.LOOSE && Math.hypot(
                        p.getFieldXInches() - bottom.getFieldXInches(), p.getFieldYInches() - bottom.getFieldYInches())
                        < p.getRadiusInches() + 0.05) return false;
            }
            TriggerVolume spot = new TriggerVolume("flower", ahead, side, 0.05, 0.05);
            if (field.tryAcquire(spot, true, POLLEN_ONLY, 1) == 0) return false;
            for (GamePiece p : field.getPieces()) {
                if (inColumn(p) && p.isSleeping() && Math.hypot(
                        p.getFieldXInches() - c[0], p.getFieldYInches() - c[1]) < SimField.FLOWER_INNER_RADIUS) {
                    SimField.setSleeping(p, false);
                }
            }
            return true;
        }
        return false;
    }

    /** On the field and not in the robot or a hive's cell: a ball that can be resting in a flower. */
    private static boolean inColumn(GamePiece p) {
        return p.getState() == PossessionState.LOOSE || p.getState() == PossessionState.IN_FLIGHT;
    }

    private void launch(int id, double x, double y, double heading, double vx, double vy, double[] aim) {
        double hood = RobotConstants.Launcher.HOOD_ANGLE;
        double r = RobotConstants.Launcher.EXIT_RADIUS;
        double ex = x + r * Math.cos(heading), ey = y + r * Math.sin(heading);
        double ez = RobotConstants.Launcher.EXIT_HEIGHT;
        double speed = Math.abs(flywheel.getVelocity()) / RobotConstants.Launcher.FLYWHEEL_TICKS_PER_EXIT_SPEED;
        speed *= 1 + speedBias;
        if (spread != null) {
            speed *= 1 + speedSpread * spread.nextGaussian();
            heading += headingSpread * spread.nextGaussian();
        }
        double horizontal = speed * Math.cos(hood);
        field.launch(id, "launcher", ex, ey, ez, horizontal * Math.cos(heading) + vx,
                horizontal * Math.sin(heading) + vy, speed * Math.sin(hood));
        shotsFired++;
        kicked = true;

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
