package org.firstinspires.ftc.teamcode.core;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.*;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Drive.INTAKE_HEADING_OFFSET;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.ROBOT_WIDTH;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BallVision;
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.firstinspires.ftc.teamcode.vision.PollenTracker;
import org.psilynx.psikit.core.Logger;

import java.util.List;

/**
 * Drives the intake over pollen the ball camera sees, one after another, with the intake running.
 *
 * Each pickup: follow a straight path that puts the intake's mouth OVERSHOOT inches past the target, facing it, and
 * redo the path whenever the target's position estimate moves. The camera can't see the last few inches in front of
 * the mouth, so a target that leaves the frame just ahead of the intake is still driven to. Once the
 * path is done, the target has to stay out of sight for CONFIRM_S to count as picked up; if it shows up again (pushed
 * or bounced) the chase goes after it again, up to MAX_TRIES times.
 *
 * Paths are capped at CHASE_MAX_SPEED, and NEAR_BALL_SPEED within NEAR_BALL_DISTANCE of the target. A flower or hive
 * foot in the way is driven around (FieldRoutes): the path goes to the first waypoint, then on. A ball beside a wall,
 * a foot or a flower is lined up first: LINE_UP_OUT out from it on its open side, then straight in, square. One within
 * WALL_RUN of the wall ahead (and not by a flower) is driven past to the wall, so balls in a row behind it (hidden behind the first) go in
 * too. A rolling
 * target is waited for. A target the robot gets no closer to for STUCK_S (pinned against a wall, say), or that sits
 * within reach for IN_REACH_S without going in, is skipped for SKIP_S.
 *
 * With nothing to chase it searches: turns in place at SEARCH_TURN_RATE, toward the side the camera looks, until pollen
 * comes into view. A frame up against a wall can't turn, so it first drives out to the nearest spot with room to turn.
 * Ends after maxBalls pickups, after a SEARCH_TURN without seeing any, or after timeoutS. The robot then holds its
 * position and the intake stays on.
 *
 * Nothing counts the balls inside, so pickups are counted from the camera, erring high: every pollen seen in the last
 * REMEMBER_S that the intake's mouth then passes over went in (a row along a wall goes in together, and the ones behind
 * the first are hidden from the camera by then), and a target confirmed gone counts at least one. A target at the
 * mouth that won't go in, FULL_REFUSALS times in a row, means the robot is full.
 */
public class PollenChase {
    private static final int MAX_TRIES = 3;
    private static final int FULL_REFUSALS = 2;
    private static final double REMEMBER_S = 2;
    // A ball this close to the wall straight ahead is driven past all the way to the wall: a row of 4 along that line
    private static final double WALL_RUN = 12;

    private enum State {
        APPROACH, // driving at a target in view
        BLIND,    // the target went out of view where it's expected to (the blind spot ahead of the mouth, or beside the
                  // camera once lined up square to a wall); finishing the path to where it was last seen
        CONFIRM   // path done; waiting to see whether the target is gone
    }

    private final Robot robot;
    private final Follower follower;
    private final BallVision vision;
    private final int maxBalls;
    private final double timeoutS;

    private State state;
    private int pickedUp;
    private double startTime, stateSince, seenBefore;
    // The search: where it turns (null while there's a target), when it set off there, and when it started turning
    // (NaN until it's there)
    private Pose searchFrom;
    private double searchBegan, turnStart = Double.NaN;
    // Progress toward the target: where the robot was when it last got anywhere, and when
    private Pose progressPose;
    private double progressTime;
    private PollenTracker.Target chasing;
    private int tries;
    private Pose goal;
    // A waypoint on the way to goal, while the robot is driving to it: around a flower or hive foot, or the line-up
    // point in front of a ball beside something (detourLinesUp)
    private Pose detour;
    private boolean detourLinesUp, linedUp;
    private double linedUpAt = Double.NEGATIVE_INFINITY; // when it got to the line-up spot
    private double pathSpeed;
    // The target's recent positions {t, x, y}, to tell a rolling ball; when it came within reach (NaN: it isn't)
    private final java.util.ArrayDeque<double[]> track = new java.util.ArrayDeque<>();
    private double inReachSince = Double.NaN;
    // Pollen seen lately, field {x, y, last seen}; how many the mouth has passed over since the last confirmed pickup;
    // targets in a row that wouldn't go in
    private final List<double[]> seenPollen = new java.util.ArrayList<>();
    private int swept, refusals;
    private double lastSweptFrame = Double.NaN, sweptFullAt = Double.NaN;
    private double moreS = Double.POSITIVE_INFINITY, lastPickupAt; // see endingAfter

    public PollenChase(Robot robot, int maxBalls, double timeoutS) {
        this.robot = robot;
        this.follower = robot.drivebase.getFollower();
        this.vision = robot.ballVision;
        this.maxBalls = maxBalls;
        this.timeoutS = timeoutS;
    }

    /** Turns the intake on, then chases. */
    public Command command() {
        return sequential(robot.intake.intake(), Command.build()
                .setStart(this::start)
                .setExecute(this::execute)
                .setDone(this::done)
                .setEnd(end -> follower.hold(follower.pose()))
                .requiring(robot.drivebase));
    }

    /**
     * Ends the chase once it has something and hasn't picked up more for this long: what it has is worth more than a
     * long hunt for the last one. Before command().
     */
    public PollenChase endingAfter(double seconds) {
        moreS = seconds;
        return this;
    }

    /** Pollen taken in, as far as it can tell (it counts high rather than low). */
    public int getPickedUp() {
        return pickedUp + swept;
    }

    /** What the chase is doing, for telemetry. */
    public String status() {
        if (state == null) return "not started";
        String doing = chasing != null ? state.toString()
                : searchFrom == null ? "no target" : Double.isNaN(turnStart) ? "moving to turn" : "search turn";
        return doing + ", picked up " + pickedUp;
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    private void start() {
        state = State.APPROACH;
        pickedUp = 0;
        startTime = now();
        searchFrom = null;
        turnStart = Double.NaN;
        progressPose = null;
        chasing = null;
        goal = null;
        detour = null;
        vision.tracker().drop();
        vision.clearSkips();
        seenPollen.clear();
        swept = 0;
        sweptFullAt = Double.NaN;
        refusals = 0;
    }

    private boolean done() {
        double t = now();
        // Full by what the mouth passed over: CONFIRM_S more with the intake on, for it to take them in
        if (getPickedUp() < maxBalls) sweptFullAt = Double.NaN;
        else if (Double.isNaN(sweptFullAt)) sweptFullAt = t;
        return pickedUp >= maxBalls || t - sweptFullAt >= CONFIRM_S || t - startTime > timeoutS
                || searched(t) >= SEARCH_TURN || (getPickedUp() > 0 && t - lastPickupAt > moreS);
    }

    private void execute() {
        double t = now();
        PollenTracker tracker = vision.tracker();
        PollenTracker.Target target = tracker.target();
        sweep(tracker.chaseable(), t);
        if (MatchLog.active()) {
            Logger.recordOutput("Chase/status", status());
            Logger.recordOutput("Chase/swept", swept);
            Logger.recordOutput("Chase/target", target == null ? "none"
                    : String.format("%.1f, %.1f%s%s", target.x, target.y, target.atIntake ? " at intake" : "",
                    linedUp ? " lined up" : ""));
        }
        if (target != chasing) {
            chasing = target;
            tries = 0;
            goal = null;
            detour = null;
            linedUp = false;
            linedUpAt = Double.NEGATIVE_INFINITY;
            track.clear();
            inReachSince = Double.NaN;
            state = State.APPROACH;
            progressPose = null;
        }
        if (target == null) {
            search(t);
            return;
        }
        searchFrom = null;
        turnStart = Double.NaN;
        boolean inView = target.lastSeen == vision.getFrameTime();
        boolean wontGoIn = inReachTooLong(target, t);
        if (wontGoIn) refused();
        if (wontGoIn || (state != State.CONFIRM && stuck(t))) {
            tracker.drop();
            detour = null;
            return;
        }
        if (state == State.APPROACH && rolling(target, t)) {
            follower.hold(follower.pose()); // let it settle
            goal = null;
            detour = null;
            progressPose = null;
            return;
        }

        switch (state) {
            case APPROACH:
                if (!inView && target.atIntake) {
                    enter(State.BLIND, target, t);
                } else if (!inView && linedUp && lineUpClose(target)) {
                    // Square to the wall, close in: the camera (turned to one side) loses a ball against the wall
                    // here. Nose in to where it was
                    driveTo(target);
                    if (detour == null) enter(State.BLIND, target, t);
                } else if (!inView && !goingToLineUp() && t - Math.max(target.lastSeen, linedUpAt) > LOST_S) {
                    // Out of view too long. Not while on the way to a line-up spot (it drives there already facing
                    // the wall, the camera turned away), and counting from getting there
                    tracker.drop();
                } else if (goal != null && detour == null && !follower.following()) {
                    enter(State.CONFIRM, target, t); // drove all the way and it's still in view
                } else {
                    driveTo(target);
                }
                break;
            case BLIND:
                if (target.lastSeen > seenBefore || detour != null) state = State.APPROACH; // back in view, or still going around
                else if (!follower.following()) enter(State.CONFIRM, target, t);
                break;
            case CONFIRM:
                if (target.lastSeen > seenBefore) {
                    Pose pose = follower.pose();
                    if (Math.hypot(target.x - pose.x(), target.y - pose.y()) < IN_REACH) refused();
                    if (pickedUp >= maxBalls) break;
                    if (++tries >= MAX_TRIES) {
                        tracker.drop();
                    } else {
                        goal = null;
                        state = State.APPROACH;
                        progressPose = null;
                    }
                } else if (t - stateSince >= CONFIRM_S) {
                    pickedUp += Math.max(1, swept); // the target is among those swept, if the camera saw it go in
                    lastPickupAt = t;
                    swept = 0;
                    refusals = 0;
                    tracker.drop();
                }
                break;
        }
    }

    /**
     * Drives to the nearest spot with room to turn (for up to SEARCH_MOVE_S), then turns in place there, SEARCH_TURN_RATE
     * toward the camera's side, to find pollen.
     */
    private void search(double t) {
        if (searchFrom == null) {
            searchFrom = roomToTurn(follower.pose());
            searchBegan = t;
        }
        if (Double.isNaN(turnStart)) {
            Pose pose = follower.pose();
            boolean there = Math.hypot(pose.x() - searchFrom.x(), pose.y() - searchFrom.y()) < 1;
            if (!there && t - searchBegan < SEARCH_MOVE_S) {
                follower.hold(searchFrom);
                return;
            }
            turnStart = t;
        }
        double turn = Math.min(searched(t), SEARCH_TURN) * (YAW >= 0 ? 1 : -1);
        follower.hold(new Pose(searchFrom.x(), searchFrom.y(), searchFrom.heading() + turn));
    }

    /** How far the current search has turned, radians; 0 when not turning. */
    private double searched(double t) {
        return Double.isNaN(turnStart) ? 0 : (t - turnStart) * SEARCH_TURN_RATE;
    }

    /**
     * Whether the robot has been trying to get to the target for STUCK_S without getting anywhere. Waiting for the
     * camera to settle doesn't count: only while a path is being followed.
     */
    private boolean stuck(double t) {
        Pose pose = follower.pose();
        if (progressPose == null || !follower.following()
                || Math.hypot(pose.x() - progressPose.x(), pose.y() - progressPose.y()) > STUCK_DISTANCE
                || Math.abs(Math.atan2(Math.sin(pose.heading() - progressPose.heading()),
                        Math.cos(pose.heading() - progressPose.heading()))) > STUCK_TURN) {
            progressPose = pose;
            progressTime = t;
            return false;
        }
        if (t - progressTime < STUCK_S) return false;
        progressPose = null;
        return true;
    }

    /**
     * The nearest spot to the pose where the robot can turn in place: its half diagonal plus KEEP_OUT_MARGIN from the
     * walls, the hive's feet and the flowers. Keeps the heading.
     */
    static Pose roomToTurn(Pose pose) {
        double r = Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2 + RobotConstants.Field.KEEP_OUT_MARGIN;
        double x = Math.max(r, Math.min(144 - r, pose.x())), y = Math.max(r, Math.min(144 - r, pose.y()));
        for (Pose flower : RobotConstants.Field.FLOWERS) {
            double dx = x - flower.x(), dy = y - flower.y(), d = Math.hypot(dx, dy), need = r + RobotConstants.Field.FLOWER_RADIUS;
            if (d >= need) continue;
            if (d < 1e-6) dx = 72 - flower.x(); // on the flower: out toward the field center
            if (d < 1e-6) dy = 72 - flower.y();
            d = Math.hypot(dx, dy);
            x = flower.x() + dx / d * need;
            y = flower.y() + dy / d * need;
        }
        for (double[] foot : RobotConstants.Field.HIVE_FEET) {
            double x0 = foot[0] - r, y0 = foot[1] - r, x1 = foot[2] + r, y1 = foot[3] + r;
            if (x <= x0 || x >= x1 || y <= y0 || y >= y1) continue;
            // Out the nearest side
            double left = x - x0, right = x1 - x, down = y - y0, up = y1 - y;
            double least = Math.min(Math.min(left, right), Math.min(down, up));
            if (least == left) x = x0;
            else if (least == right) x = x1;
            else if (least == down) y = y0;
            else y = y1;
        }
        return new Pose(Math.max(r, Math.min(144 - r, x)), Math.max(r, Math.min(144 - r, y)), pose.heading());
    }

    private void enter(State next, PollenTracker.Target target, double t) {
        state = next;
        stateSince = t;
        seenBefore = target.lastSeen;
    }

    private boolean goingToLineUp() {
        return detour != null && detourLinesUp && follower.following();
    }

    /** Whether the robot is about at the line-up spot for the target or closer in. */
    private boolean lineUpClose(PollenTracker.Target target) {
        Pose pose = follower.pose();
        return Math.hypot(target.x - pose.x(), target.y - pose.y()) < LINE_UP_OUT + LINE_UP_TOLERANCE;
    }

    /** (Re)plans a path that puts the mouth OVERSHOOT past the target, if the target moved enough to matter. */
    private void driveTo(PollenTracker.Target target) {
        Pose pose = follower.pose();
        double dx = target.x - pose.x(), dy = target.y - pose.y();
        double distance = Math.hypot(dx, dy);
        if (distance < 1e-3) return;
        double ux = dx / distance, uy = dy / distance;
        // Beside a wall, a hive foot or a flower: in square from the open side, lined up LINE_UP_OUT out first
        double[] open = besideSomething(target.x, target.y);
        Pose lineUp = null;
        if (open != null) {
            ux = -open[0];
            uy = -open[1];
            double along = (pose.x() - target.x) * open[0] + (pose.y() - target.y) * open[1];
            double across = Math.abs((pose.y() - target.y) * open[0] - (pose.x() - target.x) * open[1]);
            if (along > 0 && across < LINE_UP_TOLERANCE) linedUp = true;
        }
        double heading = Math.atan2(uy, ux) - INTAKE_HEADING_OFFSET;
        if (open != null && !linedUp) {
            lineUp = new Pose(clamp(target.x + open[0] * LINE_UP_OUT), clamp(target.y + open[1] * LINE_UP_OUT), heading);
        }
        // A wall close ahead of the ball: on to the wall (clamp stops it short), for any balls in a row behind this one
        double past = wallAhead(target.x, target.y, ux, uy) < WALL_RUN && !flowerNear(target.x, target.y)
                ? WALL_RUN : OVERSHOOT;
        double x = clamp(target.x + ux * (past - INTAKE_REACH));
        double y = clamp(target.y + uy * (past - INTAKE_REACH));
        double speed = distance < NEAR_BALL_DISTANCE ? NEAR_BALL_SPEED : CHASE_MAX_SPEED;
        if (detour != null) {
            if (follower.following()) return; // still on the way there
            if (detourLinesUp) {
                linedUp = true;
                linedUpAt = now();
            }
            detour = null;                    // there: on to the ball
            goal = null;
            if (open != null && linedUp) lineUp = null;
        }
        // Redo the path if the target moved enough, or it's come within NEAR_BALL_DISTANCE (a slower cap)
        if (goal != null && Math.hypot(x - goal.x(), y - goal.y()) < REPLAN_DISTANCE && speed == pathSpeed) return;

        goal = new Pose(x, y, heading);
        pathSpeed = speed;
        Pose next = lineUp != null ? lineUp : goal;
        List<double[]> route = FieldRoutes.between(pose, next);
        Pose end = goal;
        if (!route.isEmpty()) {
            detour = new Pose(route.get(0)[0], route.get(0)[1], heading);
            detourLinesUp = false;
            end = detour;
        } else if (lineUp != null) {
            detour = lineUp;
            detourLinesUp = true;
            end = lineUp;
        }
        follower.follow(line(pose, end).constant(heading)
                .with(Constants.foresightConfig.maxVelocityConstraint.at(end == goal ? speed : CHASE_MAX_SPEED)));
    }

    /**
     * The unit direction, {x, y}, away from the wall, hive foot or flower column the ball is within LINE_UP_DISTANCE
     * of (the nearest), toward the open side; null if it's in the open.
     */
    static double[] besideSomething(double x, double y) {
        double best = LINE_UP_DISTANCE;
        double[] open = null;
        double[][] walls = {{x, 1, 0}, {144 - x, -1, 0}, {y, 0, 1}, {144 - y, 0, -1}}; // distance, then the way out
        for (double[] w : walls) {
            if (w[0] < best) {
                best = w[0];
                open = new double[] {w[1], w[2]};
            }
        }
        for (double[] foot : RobotConstants.Field.HIVE_FEET) {
            double qx = Math.max(foot[0], Math.min(foot[2], x)), qy = Math.max(foot[1], Math.min(foot[3], y));
            double d = Math.hypot(x - qx, y - qy);
            if (d > 1e-6 && d < best) {
                best = d;
                open = new double[] {(x - qx) / d, (y - qy) / d};
            }
        }
        for (Pose flower : RobotConstants.Field.FLOWERS) {
            double r = Math.hypot(x - flower.x(), y - flower.y());
            if (r > 1e-6 && r - RobotConstants.Field.FLOWER_RADIUS < best) {
                best = r - RobotConstants.Field.FLOWER_RADIUS;
                open = new double[] {(x - flower.x()) / r, (y - flower.y()) / r};
            }
        }
        return open;
    }

    /**
     * Notes the pollen in this frame (once per frame), forgets what hasn't been seen for REMEMBER_S, and counts as
     * swept in any the mouth is now over: just inside the bumper to past INTAKE_REACH, between the side plates.
     */
    private void sweep(List<double[]> pollen, double t) {
        double frame = vision.getFrameTime();
        if (frame != lastSweptFrame) {
            lastSweptFrame = frame;
            for (double[] p : pollen) {
                double[] known = null;
                for (double[] q : seenPollen) if (Math.hypot(p[0] - q[0], p[1] - q[1]) < 1.5) known = q;
                if (known == null) seenPollen.add(new double[] {p[0], p[1], t});
                else {
                    known[0] = p[0];
                    known[1] = p[1];
                    known[2] = t;
                }
            }
        }
        Pose pose = follower.pose();
        double c = Math.cos(pose.heading() + INTAKE_HEADING_OFFSET), s = Math.sin(pose.heading() + INTAKE_HEADING_OFFSET);
        double r = Ball.Type.POLLEN.diameter / 2;
        for (java.util.Iterator<double[]> it = seenPollen.iterator(); it.hasNext(); ) {
            double[] p = it.next();
            if (t - p[2] > REMEMBER_S) {
                it.remove();
                continue;
            }
            double dx = p[0] - pose.x(), dy = p[1] - pose.y();
            double ahead = dx * c + dy * s, side = -dx * s + dy * c;
            if (ahead > ROBOT_LENGTH / 2 - 1 && ahead < INTAKE_REACH + r && Math.abs(side) < INTAKE_WIDTH / 2) {
                swept++;
                lastPickupAt = t;
                it.remove();
            }
        }
    }

    /** A target at the mouth didn't go in: after FULL_REFUSALS of those in a row, the robot is taken as full. */
    private void refused() {
        if (++refusals >= FULL_REFUSALS) pickedUp = Math.max(pickedUp, maxBalls);
    }

    /** Whether a flower is within WALL_RUN and half the robot's width of (x, y): no running on to the wall there. */
    static boolean flowerNear(double x, double y) {
        for (Pose flower : RobotConstants.Field.FLOWERS) {
            if (Math.hypot(x - flower.x(), y - flower.y()) < WALL_RUN + ROBOT_WIDTH / 2) return true;
        }
        return false;
    }

    /** How far from (x, y), going (ux, uy), to the wall. */
    static double wallAhead(double x, double y, double ux, double uy) {
        double d = Double.POSITIVE_INFINITY;
        if (ux > 1e-6) d = Math.min(d, (144 - x) / ux);
        if (ux < -1e-6) d = Math.min(d, -x / ux);
        if (uy > 1e-6) d = Math.min(d, (144 - y) / uy);
        if (uy < -1e-6) d = Math.min(d, -y / uy);
        return d;
    }

    /** Whether the target has rolled more than MOVING_SPEED, net, over the last MOVING_WINDOW_S. */
    private boolean rolling(PollenTracker.Target target, double t) {
        track.addLast(new double[] {t, target.x, target.y});
        while (track.size() > 1 && t - track.peekFirst()[0] > MOVING_WINDOW_S) track.removeFirst();
        double[] first = track.peekFirst();
        double span = t - first[0];
        return span >= MOVING_WINDOW_S / 2
                && Math.hypot(target.x - first[1], target.y - first[2]) / span > MOVING_SPEED;
    }

    /** Whether the target has been within IN_REACH of the robot's center for IN_REACH_S without being picked up. */
    private boolean inReachTooLong(PollenTracker.Target target, double t) {
        Pose pose = follower.pose();
        if (Math.hypot(target.x - pose.x(), target.y - pose.y()) >= IN_REACH) {
            inReachSince = Double.NaN;
            return false;
        }
        if (Double.isNaN(inReachSince)) inReachSince = t;
        return t - inReachSince >= IN_REACH_S;
    }

    /** Keeps path ends inside the walls, by half the robot's longer side. */
    private static double clamp(double v) {
        double half = Math.max(ROBOT_WIDTH, ROBOT_LENGTH) / 2;
        return Math.max(half, Math.min(144 - half, v));
    }
}
