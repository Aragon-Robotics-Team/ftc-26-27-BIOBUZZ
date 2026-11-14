package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.SHOT_TABLE_EXTRAPOLATE;

import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Flywheel speed by distance, measured on the real robot: each point is a spot where standing shots went through the
 * middle of the opening at that flywheel speed. SolversLib's {@link InterpLUT} draws a smooth curve through the points
 * (a monotone cubic spline: it never overshoots between two points); past either end it holds the end point's speed.
 *
 * <p>The points live here in code. To fill it: run the Shot Table Calibration OpMode, which shows each save as a
 * {@code {distance, speed},} line, and paste those lines into POINTS below, in any order. It needs at least two
 * different distances; until then the Launcher sets the flywheel from the shot model instead.
 *
 * <p>Distance is horizontal inches from the turret pivot to the center of the hive cell's opening, as
 * {@code Launcher.distanceTo} measures it.
 */
public final class ShotTable {
    /** {distance (inches), flywheel speed (ticks/sec)}. */
    static final double[][] POINTS = {
            // Paste from Shot Table Calibration here, e.g. {41.5, 840},
    };

    private static final Table TABLE = new Table(POINTS);

    private ShotTable() {}

    /** True if there aren't enough points for a table (two different distances). */
    public static boolean isEmpty() {
        return TABLE.lut == null;
    }

    /** True if the distance is inside the table, or no more than SHOT_TABLE_EXTRAPOLATE past either end. */
    public static boolean covers(double distance) {
        return TABLE.covers(distance);
    }

    /** Flywheel ticks/sec for this distance (the end point's past either end), within 0 to FLYWHEEL_MAX_VELOCITY. NaN if empty. */
    public static double velocityAt(double distance) {
        return TABLE.velocityAt(distance);
    }

    /** The number of distinct distances in the table, for telemetry. */
    public static int size() {
        return TABLE.size;
    }

    /**
     * POINTS made into an InterpLUT. InterpLUT throws on fewer than two points or distances out of order, and this
     * runs when the class loads, so it sorts them and averages any at the same distance first: a paste can't stop
     * the robot code from starting.
     */
    public static final class Table {
        final InterpLUT lut; // null with fewer than two distances
        final double min, max;
        final int size;

        public Table(double[][] points) {
            double[][] sorted = points.clone();
            Arrays.sort(sorted, (a, b) -> Double.compare(a[0], b[0]));
            List<double[]> merged = new ArrayList<>();
            int i = 0;
            while (i < sorted.length) {
                double x = sorted[i][0], sum = 0;
                int n = 0;
                while (i < sorted.length && sorted[i][0] == x) {
                    sum += sorted[i][1];
                    n++;
                    i++;
                }
                merged.add(new double[] {x, sum / n});
            }
            size = merged.size();
            min = size > 0 ? merged.get(0)[0] : Double.NaN;
            max = size > 0 ? merged.get(size - 1)[0] : Double.NaN;
            if (size < 2) {
                lut = null;
                return;
            }
            InterpLUT built = new InterpLUT();
            for (double[] p : merged) built.add(p[0], p[1]);
            lut = built.createLUT();
        }

        public boolean covers(double distance) {
            return lut != null && distance >= min - SHOT_TABLE_EXTRAPOLATE && distance <= max + SHOT_TABLE_EXTRAPOLATE;
        }

        public double velocityAt(double distance) {
            if (lut == null) return Double.NaN;
            return Math.max(0, Math.min(lut.get(distance), FLYWHEEL_MAX_VELOCITY));
        }
    }
}
