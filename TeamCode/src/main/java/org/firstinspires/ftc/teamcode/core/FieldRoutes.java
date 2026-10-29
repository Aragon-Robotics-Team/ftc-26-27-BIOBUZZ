package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.FLOWERS;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.FLOWER_RADIUS;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.HIVE_FEET;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.KEEP_OUT_MARGIN;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Field.ROBOT_WIDTH;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * Waypoints for a straight drive that would hit a flower column or one of the hive's feet. A line is in the way when it
 * passes closer than PASS (the robot's wider half plus an inch) to one; the waypoint around it is CLEARANCE out (the
 * robot's half diagonal plus KEEP_OUT_MARGIN), where the robot can turn. A drive that starts or ends that close to one is
 * left alone: it's going to a ball beside it. Pedro coordinates, inches. Flowers and feet are symmetric about the field
 * center, so this works for both alliances. No hardware, so it's unit tested.
 */
public final class FieldRoutes {
    /** How close the robot's center may pass a flower column's outside or a hive foot. */
    public static final double PASS = Math.max(ROBOT_LENGTH, ROBOT_WIDTH) / 2 + 1;
    /** How far a waypoint is from a flower column's outside or a hive foot. */
    public static final double CLEARANCE = Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2 + KEEP_OUT_MARGIN;
    private static final int MAX_SPLITS = 4;

    private FieldRoutes() {}

    /** The waypoints, {x, y} each, to drive through on the way from a to b; empty if the straight line is clear. */
    public static List<double[]> between(Pose a, Pose b) {
        List<double[]> route = new ArrayList<>();
        add(route, a.x(), a.y(), b.x(), b.y(), MAX_SPLITS);
        return route;
    }

    /** Adds the waypoints from (ax, ay) to (bx, by): around the first obstacle, then each leg the same way. */
    private static void add(List<double[]> route, double ax, double ay, double bx, double by, int splits) {
        if (splits == 0) return;
        double[] via = around(ax, ay, bx, by);
        // None, or one where it already is (no way found): drive straight
        if (via == null || Math.hypot(via[0] - ax, via[1] - ay) < 1) return;
        add(route, ax, ay, via[0], via[1], splits - 1);
        route.add(via);
        add(route, via[0], via[1], bx, by, splits - 1);
    }

    /** A waypoint around the first obstacle on the line from (ax, ay) to (bx, by), or null if it's clear. */
    private static double[] around(double ax, double ay, double bx, double by) {
        for (double[] foot : HIVE_FEET) {
            if (inBox(ax, ay, foot, PASS) || inBox(bx, by, foot, PASS)) continue;
            if (!crosses(ax, ay, bx, by, foot[0] - PASS, foot[1] - PASS, foot[2] + PASS, foot[3] + PASS)) continue;
            double y, x;
            if (ay <= foot[1] - PASS || ay >= foot[3] + PASS) {
                // a is already past an end of the foot: round that end to b's side
                y = ay <= foot[1] - PASS ? foot[1] - CLEARANCE : foot[3] + CLEARANCE;
                x = bx < (foot[0] + foot[2]) / 2 ? foot[0] - CLEARANCE : foot[2] + CLEARANCE;
            } else {
                // a is beside it: past the end nearer the middle of the drive, on a's side
                y = (ay + by) / 2 < (foot[1] + foot[3]) / 2 ? foot[1] - CLEARANCE : foot[3] + CLEARANCE;
                x = ax < (foot[0] + foot[2]) / 2 ? foot[0] - CLEARANCE : foot[2] + CLEARANCE;
            }
            return new double[] {onField(x), onField(y)};
        }
        for (Pose flower : FLOWERS) {
            double pass = FLOWER_RADIUS + PASS;
            if (Math.hypot(ax - flower.x(), ay - flower.y()) < pass || Math.hypot(bx - flower.x(), by - flower.y()) < pass) {
                continue;
            }
            double[] q = closestOnSegment(flower.x(), flower.y(), ax, ay, bx, by);
            double dx = q[0] - flower.x(), dy = q[1] - flower.y(), d = Math.hypot(dx, dy);
            if (d >= pass) continue;
            if (d < 1e-6) { // straight through its middle: past it on the field side (flowers stand by the walls)
                dx = 72 - flower.x();
                dy = 72 - flower.y();
                d = Math.hypot(dx, dy);
            }
            double r = FLOWER_RADIUS + CLEARANCE;
            return new double[] {onField(flower.x() + dx / d * r), onField(flower.y() + dy / d * r)};
        }
        return null;
    }

    private static boolean inBox(double x, double y, double[] box, double grow) {
        return x > box[0] - grow && x < box[2] + grow && y > box[1] - grow && y < box[3] + grow;
    }

    /** Keeps a waypoint CLEARANCE inside the walls. */
    private static double onField(double v) {
        return Math.max(CLEARANCE, Math.min(144 - CLEARANCE, v));
    }

    /** Distance from (px, py) to the segment from (ax, ay) to (bx, by). */
    static double distanceToSegment(double px, double py, double ax, double ay, double bx, double by) {
        double[] q = closestOnSegment(px, py, ax, ay, bx, by);
        return Math.hypot(px - q[0], py - q[1]);
    }

    /** The point of the segment from (ax, ay) to (bx, by) closest to (px, py). */
    private static double[] closestOnSegment(double px, double py, double ax, double ay, double bx, double by) {
        double dx = bx - ax, dy = by - ay, lengthSq = dx * dx + dy * dy;
        double t = lengthSq == 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / lengthSq));
        return new double[] {ax + t * dx, ay + t * dy};
    }

    /** Whether the segment from (ax, ay) to (bx, by) passes through the box (Liang–Barsky clipping). */
    static boolean crosses(double ax, double ay, double bx, double by, double x0, double y0, double x1, double y1) {
        double t0 = 0, t1 = 1, dx = bx - ax, dy = by - ay;
        double[] p = {-dx, dx, -dy, dy}, q = {ax - x0, x1 - ax, ay - y0, y1 - ay};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) return false;
                continue;
            }
            double t = q[i] / p[i];
            if (p[i] < 0) t0 = Math.max(t0, t);
            else t1 = Math.min(t1, t);
            if (t0 > t1) return false;
        }
        return true;
    }
}
