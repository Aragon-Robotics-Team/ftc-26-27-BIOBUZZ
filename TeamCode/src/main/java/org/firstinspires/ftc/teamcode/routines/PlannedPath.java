package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.interpolator.PiecewiseInterpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.WeakHashMap;

/**
 * Builds paths copied from tools/path-planner.html. Each copied path is a short method that calls {@link #of}; paste
 * it into a routine and call it with the routine's poseFactory, e.g. {@code follow(follower, gardenCycle(poseFactory))}.
 * To change a path, paste its code (from the "// path-planner" line down) back into the planner.
 */
public final class PlannedPath {
    /** A point along a planned path, for {@code AutoRoutine.passed()}: segment index and curve parameter. */
    public static final class Marker {
        public final int segment;
        public final double t;

        public Marker(int segment, double t) {
            this.segment = segment;
            this.t = t;
        }
    }

    /** A planned path as last built, with where it starts. */
    public static final class Built {
        public final Path path;
        public final Pose start;

        Built(Path path, Pose start) {
            this.path = path;
            this.start = start;
        }
    }

    private static final Map<Path, String> IDS = Collections.synchronizedMap(new WeakHashMap<Path, String>());
    private static final Map<String, Built> BUILT = Collections.synchronizedMap(new LinkedHashMap<String, Built>());

    private PlannedPath() {}

    /**
     * One planned path as a Pedro compound path, flipped for the alliance by {@code f}.
     *
     * @param id         "name#geometry", written to path logs so the planner can match runs to this path
     * @param stopAtEnd  false keeps the robot moving into whatever follows (no braking at the end)
     * @param segments   one array per cubic Bezier: x0, y0, … x3, y3 (red side), then pairs of heading breakpoint
     *                   (fraction of the segment's length) and heading (red side, degrees), swept linearly between
     */
    public static Path of(String id, PoseFactory f, boolean stopAtEnd, double[]... segments) {
        Path[] parts = new Path[segments.length];
        for (int s = 0; s < segments.length; s++) {
            double[] d = segments[s];
            Pose[] points = new Pose[4];
            for (int i = 0; i < 4; i++) points[i] = f.of(d[2 * i], d[2 * i + 1], 0);
            PiecewiseInterpolator heading = Interpolator.piecewise();
            for (int k = 10; k < d.length; k += 2) {
                heading = heading.until(d[k], Interpolator.linear(f.of(0, 0, d[k - 1]).heading(), f.of(0, 0, d[k + 1]).heading()));
            }
            parts[s] = Paths.curve(points).heading(heading);
        }
        Path path = Paths.path(parts);
        if (!stopAtEnd) path = path.with(Constants.foresightConfig.brakeAtEnd.at(false));
        IDS.put(path, id);
        BUILT.put(id.split("#")[0], new Built(path, f.of(segments[0][0], segments[0][1], segments[0][9])));
        return path;
    }

    /** The id a planned path was built with, or null for any other path. */
    public static String idOf(Path path) {
        return path == null ? null : IDS.get(path);
    }

    /** Every planned path built so far in this app run, by name (the latest build of each). */
    public static Map<String, Built> built() {
        synchronized (BUILT) {
            return new LinkedHashMap<>(BUILT);
        }
    }

    /** Forgets the paths built so far, e.g. before rebuilding them for the other alliance. */
    public static void clearBuilt() {
        BUILT.clear();
    }
}
