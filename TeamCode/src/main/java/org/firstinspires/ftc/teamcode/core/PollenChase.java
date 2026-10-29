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
import org.firstinspires.ftc.teamcode.vision.PollenTracker;

/**
 * Drives the intake over pollen the ball camera sees, one after another, with the intake running.
 *
 * Each pickup: follow a straight path that puts the intake's mouth OVERSHOOT inches past the target, facing it, and
 * redo the path whenever the target's position estimate moves. The camera can't see the last few inches in front of
 * the mouth, so a target that leaves the frame just ahead of the intake is still driven to. Once the
 * path is done, the target has to stay out of sight for CONFIRM_S to count as picked up; if it shows up again (pushed
 * or bounced) the chase goes after it again, up to MAX_TRIES times.
 *
 * A target the robot gets no closer to for STUCK_S (pinned against a wall, say) is skipped for the rest of the chase.
 *
 * With nothing to chase it searches: turns in place at SEARCH_TURN_RATE, toward the side the camera looks, until pollen
 * comes into view. A frame up against a wall can't turn, so it first drives out to the nearest spot with room to turn.
 * Ends after maxBalls pickups, after a SEARCH_TURN without seeing any, or after timeoutS. The robot then holds its
 * position and the intake stays on.
 */
public class PollenChase {
    private static final int MAX_TRIES = 3;

    private enum State {
        APPROACH, // driving at a target in view
        BLIND,    // the target went under the camera into the blind spot ahead of the mouth; finishing the path
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

    public int getPickedUp() {
        return pickedUp;
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
        vision.tracker().drop();
        vision.clearSkips();
    }

    private boolean done() {
        double t = now();
        return pickedUp >= maxBalls || t - startTime > timeoutS || searched(t) >= SEARCH_TURN;
    }

    private void execute() {
        double t = now();
        PollenTracker tracker = vision.tracker();
        PollenTracker.Target target = tracker.target();
        if (target != chasing) {
            chasing = target;
            tries = 0;
            goal = null;
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
        if (state != State.CONFIRM && stuck(t)) {
            vision.skip(target.x, target.y);
            tracker.drop();
            return;
        }

        switch (state) {
            case APPROACH:
                if (!inView && target.atIntake) {
                    enter(State.BLIND, target, t);
                } else if (!inView && t - target.lastSeen > LOST_S) {
                    tracker.drop();
                } else if (goal != null && !follower.following()) {
                    enter(State.CONFIRM, target, t); // drove all the way and it's still in view
                } else {
                    driveTo(target);
                }
                break;
            case BLIND:
                if (target.lastSeen > seenBefore) state = State.APPROACH; // it came back into view
                else if (!follower.following()) enter(State.CONFIRM, target, t);
                break;
            case CONFIRM:
                if (target.lastSeen > seenBefore) {
                    if (++tries >= MAX_TRIES) {
                        tracker.drop();
                    } else {
                        goal = null;
                        state = State.APPROACH;
                        progressPose = null;
                    }
                } else if (t - stateSince >= CONFIRM_S) {
                    pickedUp++;
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

    /** (Re)plans a path that puts the mouth OVERSHOOT past the target, if the target moved enough to matter. */
    private void driveTo(PollenTracker.Target target) {
        Pose pose = follower.pose();
        double dx = target.x - pose.x(), dy = target.y - pose.y();
        double distance = Math.hypot(dx, dy);
        if (distance < 1e-3) return;
        double ux = dx / distance, uy = dy / distance;
        double x = clamp(target.x + ux * (OVERSHOOT - INTAKE_REACH));
        double y = clamp(target.y + uy * (OVERSHOOT - INTAKE_REACH));
        if (goal != null && Math.hypot(x - goal.x(), y - goal.y()) < REPLAN_DISTANCE) return;

        double heading = Math.atan2(uy, ux) - INTAKE_HEADING_OFFSET;
        goal = new Pose(x, y, heading);
        follower.follow(line(pose, goal).constant(heading)
                .with(Constants.foresightConfig.maxPathSpeed.at(CHASE_MAX_SPEED)));
    }

    /** Keeps path ends inside the walls, by half the robot's longer side. */
    private static double clamp(double v) {
        double half = Math.max(ROBOT_WIDTH, ROBOT_LENGTH) / 2;
        return Math.max(half, Math.min(144 - half, v));
    }
}
