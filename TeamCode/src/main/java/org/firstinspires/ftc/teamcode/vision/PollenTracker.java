package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Picks which pollen to chase and keeps that choice steady from frame to frame. Works in field coordinates, so the
 * target stays put while the robot drives (the pixels don't). Plain Java; BallVision feeds it each new frame with the
 * robot pose from when the frame was taken.
 *
 * The closest chaseable pollen gets locked. It stays the target while a pollen keeps showing up near it; another one
 * takes over only after being clearly closer (switchMargin x the locked distance) for switchFrames frames in a row.
 * The tracker never lets go of a target by itself: the chase decides when it's picked up or lost and calls
 * {@link #drop()}.
 */
public class PollenTracker {
    /** The robot's values are in RobotConstants.BallVision. */
    public static class Settings {
        public double lockGate;        // inches: a pollen this close to the target's last spot is the target
        public double smoothing;       // weight of each new sighting in the target's position
        public double switchMargin;
        public int switchFrames;
        public double intakeReach;     // inches from the robot center forward to the intake's mouth
        public double intakeHalfWidth; // inches either side of the intake's center line
        public double intakeAxisLeft;  // inches left of the robot center the intake's center line runs
        public double nearMouth;       // inches ahead of the mouth that count as about to go in
        public double edgeMarginPx;    // a ball this close to a frame edge is leaving the frame
    }

    /** Decides which field positions may be chased (inside the walls, off the keep-outs). */
    public interface Area {
        boolean allows(double x, double y);
    }

    /** The pollen being chased. */
    public static class Target {
        public double x, y;        // field, inches (smoothed)
        public double lastSeen;    // capture time of the last frame it was in, seconds
        /**
         * In its last sighting it was leaving the frame just ahead of the mouth: once it's out of view it's in the
         * camera's blind spot in front of the intake, not lost. (Which edge depends on the mount: the right-side
         * camera loses a ball straight ahead off its bottom edge.)
         */
        public boolean atIntake;
        public Ball lastBall;      // pixels, in its last sighting
    }

    private final Settings settings;
    private final Area area;
    private Target target;
    private Target challenger;
    private int challengerFrames;
    private List<double[]> chaseable = Collections.emptyList();

    public PollenTracker(Settings settings, Area area) {
        this.settings = settings;
        this.area = area;
    }

    /**
     * One new frame.
     *
     * @param time  capture time, seconds
     * @param robot robot pose when the frame was taken: field x, y (inches) and heading (radians)
     */
    public void update(double time, double[] robot, List<Ball> balls, BallGeometry geometry) {
        double cos = Math.cos(robot[2]), sin = Math.sin(robot[2]);
        List<Target> seen = new ArrayList<>();
        List<double[]> positions = new ArrayList<>();
        for (Ball ball : balls) {
            if (ball.type != Ball.Type.POLLEN) continue;
            double[] p = geometry.robotPosition(ball.x, ball.y, ball.type.diameter);
            if (p == null) continue;
            Target t = new Target();
            t.x = robot[0] + p[0] * cos - p[1] * sin;
            t.y = robot[1] + p[0] * sin + p[1] * cos;
            if (!area.allows(t.x, t.y)) continue;
            t.lastSeen = time;
            t.lastBall = ball;
            double m = settings.edgeMarginPx;
            boolean atEdge = ball.x - ball.radius <= m || ball.y - ball.radius <= m
                    || ball.x + ball.radius >= geometry.width - m || ball.y + ball.radius >= geometry.height - m;
            t.atIntake = atEdge && p[0] - settings.intakeReach <= settings.nearMouth
                    && Math.abs(p[1] - settings.intakeAxisLeft) <= settings.intakeHalfWidth;
            seen.add(t);
            positions.add(new double[]{t.x, t.y});
        }
        chaseable = positions;

        Target match = target == null ? null : nearest(seen, target.x, target.y, settings.lockGate);
        if (match != null) {
            double a = settings.smoothing;
            target.x += a * (match.x - target.x);
            target.y += a * (match.y - target.y);
            target.lastSeen = time;
            target.lastBall = match.lastBall;
            target.atIntake = match.atIntake;
        }

        Target closest = null;
        double closestDistance = Double.POSITIVE_INFINITY;
        for (Target t : seen) {
            if (t == match) continue;
            double d = Math.hypot(t.x - robot[0], t.y - robot[1]);
            if (d < closestDistance) {
                closest = t;
                closestDistance = d;
            }
        }
        if (target == null) {
            target = closest;
            challenger = null;
            return;
        }
        double targetDistance = Math.hypot(target.x - robot[0], target.y - robot[1]);
        if (closest == null || closestDistance >= settings.switchMargin * targetDistance) {
            challenger = null;
            return;
        }
        if (challenger != null && Math.hypot(closest.x - challenger.x, closest.y - challenger.y) < settings.lockGate) {
            challengerFrames++;
        } else {
            challengerFrames = 1;
        }
        challenger = closest;
        if (challengerFrames >= settings.switchFrames) {
            target = challenger;
            challenger = null;
        }
    }

    private static Target nearest(List<Target> targets, double x, double y, double within) {
        Target best = null;
        for (Target t : targets) {
            double d = Math.hypot(t.x - x, t.y - y);
            if (d < within) {
                best = t;
                within = d;
            }
        }
        return best;
    }

    /** The pollen being chased, or null. */
    public Target target() {
        return target;
    }

    /** Forget the target (picked up, or lost); the closest pollen in the next frame takes over. */
    public void drop() {
        target = null;
        challenger = null;
    }

    /** Field positions {x, y} of the chaseable pollen in the last frame. */
    public List<double[]> chaseable() {
        return chaseable;
    }
}
