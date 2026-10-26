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
 * Ends after maxBalls pickups, when no pollen has been in view for GIVE_UP_S, or after timeoutS. The robot then holds
 * its position and the intake stays on.
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
    private double startTime, stateSince, lastTargetTime, seenBefore;
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
        return state == null ? "not started" : state + (chasing == null ? " (no target)" : "") + ", picked up " + pickedUp;
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    private void start() {
        state = State.APPROACH;
        pickedUp = 0;
        startTime = now();
        lastTargetTime = startTime;
        chasing = null;
        goal = null;
        vision.tracker().drop();
    }

    private boolean done() {
        double t = now();
        return pickedUp >= maxBalls || t - startTime > timeoutS || t - lastTargetTime > GIVE_UP_S;
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
        }
        if (target == null) {
            if (!follower.holding()) follower.hold(follower.pose());
            return;
        }
        lastTargetTime = t;
        boolean inView = target.lastSeen == vision.getFrameTime();

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
                    }
                } else if (t - stateSince >= CONFIRM_S) {
                    pickedUp++;
                    tracker.drop();
                }
                break;
        }
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
