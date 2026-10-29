package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.Localization.*;

import java.util.ArrayDeque;
import java.util.Iterator;

/**
 * An extended Kalman filter that fuses odometry with the hive's AprilTags as the Limelight sees them. No hardware, so
 * it's unit tested; pedro.FusedLocalizer runs it on the robot.
 *
 * State: the robot's pose (Pedro x, y in inches, heading in radians) and the arm angle of each hive (HiveTags). The
 * tags only ever move along their circles about the arm's pivot, so one angle per hive says where all eight are; the
 * filter works it out from the tags themselves, so a hive that settles a few degrees short of its stop doesn't pull
 * the pose off.
 *
 * Odometry (the Pinpoint's motion since the last loop, in the robot's frame) moves the pose and grows its uncertainty:
 * with distance driven and angle turned, and the heading also with time, for the IMU drifting while the robot shakes.
 * Each tag the Limelight sees is a measurement: where it is in the camera's frame. The camera is on the turret, so its
 * pose comes from the robot's pose and the turret angle when the frame was taken. A tag the filter's state can't
 * explain (beyond GATE) is thrown out. When a tag's height says its hive has tipped (ARM_JUMP from the estimate),
 * that hive's angle starts over from the tag.
 *
 * Frames come in late (the Limelight's latency, then the poll), so the filter keeps its last HISTORY steps: a frame is
 * applied at the step it was taken, and the odometry since is applied again on top.
 */
public final class PoseFilter {
    /** State indices. */
    public static final int X = 0, Y = 1, HEADING = 2, RED_ARM = 3, BLUE_ARM = 4;
    private static final int N = 5;

    /** Where the Limelight is on the robot. */
    public static final class Camera {
        final double turretX, turretY, ahead, height, tilt;

        /**
         * @param turretX turret axis, inches forward of the robot center
         * @param turretY turret axis, inches left of the robot center
         * @param ahead   lens, inches ahead of the turret axis (it turns with the turret)
         * @param height  lens, inches above the tiles
         * @param tilt    radians above horizontal
         */
        public Camera(double turretX, double turretY, double ahead, double height, double tilt) {
            this.turretX = turretX;
            this.turretY = turretY;
            this.ahead = ahead;
            this.height = height;
            this.tilt = tilt;
        }
    }

    private static final class Step {
        final double time;
        final double dx, dy, dh, dt; // odometry over the step, robot frame at its start
        final double[] state = new double[N];
        final double[][] cov = new double[N][N];

        Step(double time, double dx, double dy, double dh, double dt) {
            this.time = time;
            this.dx = dx;
            this.dy = dy;
            this.dh = dh;
            this.dt = dt;
        }
    }

    private final Camera camera;
    private final double[] state = new double[N];
    private final double[][] cov = new double[N][N];
    private final ArrayDeque<Step> history = new ArrayDeque<>();
    private double since = Double.NEGATIVE_INFINITY;
    private int used, rejected, armResets;
    private final int[] armDisagree = new int[2];
    private final double[] lastSeen = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};

    public PoseFilter(Camera camera, double x, double y, double heading, double redArm, double blueArm) {
        this.camera = camera;
        state[RED_ARM] = redArm;
        state[BLUE_ARM] = blueArm;
        cov[RED_ARM][RED_ARM] = START_ARM_SIGMA * START_ARM_SIGMA;
        cov[BLUE_ARM][BLUE_ARM] = START_ARM_SIGMA * START_ARM_SIGMA;
        setPose(x, y, heading);
    }

    /**
     * Puts the robot at a known pose (a match start, a corner reset) at a time: sure of it, and forgets the history.
     * Frames taken before then are thrown out.
     */
    public void setPose(double time, double x, double y, double heading) {
        since = time;
        state[X] = x;
        state[Y] = y;
        state[HEADING] = heading;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < N; j++) {
                cov[i][j] = 0;
                cov[j][i] = 0;
            }
        }
        cov[X][X] = cov[Y][Y] = START_POSITION_SIGMA * START_POSITION_SIGMA;
        cov[HEADING][HEADING] = START_HEADING_SIGMA * START_HEADING_SIGMA;
        history.clear();
    }

    private void setPose(double x, double y, double heading) {
        setPose(Double.NEGATIVE_INFINITY, x, y, heading);
    }

    /** A hive's arm is known to be at its rest angle for this cell (a new match, or the operator says so). */
    public void setArm(int hive, double angle) {
        int a = RED_ARM + hive;
        state[a] = angle;
        for (int j = 0; j < N; j++) {
            cov[a][j] = 0;
            cov[j][a] = 0;
        }
        cov[a][a] = START_ARM_SIGMA * START_ARM_SIGMA;
    }

    // ---- Odometry ----

    /**
     * The robot moved (dx forward, dy left, dh CCW: its odometry since the last call, in its frame at the start)
     * over dt seconds, ending at time (seconds, the clock frames are stamped with).
     */
    public void predict(double time, double dx, double dy, double dh, double dt) {
        Step step = new Step(time, dx, dy, dh, dt);
        predict(state, cov, step);
        copy(state, cov, step);
        history.addLast(step);
        while (history.size() > HISTORY) history.removeFirst();
    }

    private static void predict(double[] s, double[][] p, Step step) {
        double h = s[HEADING], c = Math.cos(h), sn = Math.sin(h);
        s[X] += step.dx * c - step.dy * sn;
        s[Y] += step.dx * sn + step.dy * c;
        s[HEADING] = wrap(h + step.dh);
        // F = I, except how the move depends on the heading it was made at
        double[][] f = identity();
        f[X][HEADING] = -step.dx * sn - step.dy * c;
        f[Y][HEADING] = step.dx * c - step.dy * sn;
        double[][] next = multiply(multiply(f, p), transpose(f));
        // Odometry slips with distance, along and across the move alike
        double moved = Math.hypot(step.dx, step.dy);
        next[X][X] += TRANSLATION_DRIFT * TRANSLATION_DRIFT * moved;
        next[Y][Y] += TRANSLATION_DRIFT * TRANSLATION_DRIFT * moved;
        next[HEADING][HEADING] += HEADING_DRIFT_PER_RADIAN * HEADING_DRIFT_PER_RADIAN * Math.abs(step.dh)
                + HEADING_DRIFT_PER_S * HEADING_DRIFT_PER_S * step.dt;
        next[RED_ARM][RED_ARM] += ARM_DRIFT_PER_S * ARM_DRIFT_PER_S * step.dt;
        next[BLUE_ARM][BLUE_ARM] += ARM_DRIFT_PER_S * ARM_DRIFT_PER_S * step.dt;
        set(p, next);
    }

    // ---- Tags ----

    /**
     * One tag in one Limelight frame.
     *
     * @param time   when the frame was taken (seconds, the same clock as predict)
     * @param turret turret angle then, radians, robot-relative, CCW positive
     * @param cx     tag center in the camera's frame, inches: +x right, +y down, +z out of the lens
     * @return whether it was used: false when it isn't a hive tag, is too old, or doesn't fit
     */
    public boolean addTag(double time, double turret, int id, double cx, double cy, double cz) {
        int hive = HiveTags.hiveOf(id);
        if (hive < 0 || time < since) return false;
        lastSeen[hive] = Math.max(lastSeen[hive], time);
        // The step the frame was taken at (or the oldest kept), and the steps since
        Step base = null;
        int after = 0;
        for (Iterator<Step> it = history.descendingIterator(); it.hasNext(); ) {
            Step step = it.next();
            if (step.time <= time) {
                base = step;
                break;
            }
            after++;
        }
        if (base == null && after > 0 && history.peekFirst().time - time > MAX_AGE_S) return false;
        double[] s = new double[N];
        double[][] p = new double[N][N];
        if (base != null) {
            System.arraycopy(base.state, 0, s, 0, N);
            set(p, base.cov);
        } else {
            System.arraycopy(state, 0, s, 0, N);
            set(p, cov);
            after = 0; // nothing kept that old: apply it to now
        }

        double[] z = {cx, cy, cz};
        int arm = RED_ARM + hive;
        // A tipped hive: its tags are far from where the arm should have them, ARM_JUMP_TAGS in a row (one far tag is
        // more likely a misread). Start the arm over from the tag's height
        double fromHeight = armFromHeight(id, camera.height + cz * Math.sin(camera.tilt) - cy * Math.cos(camera.tilt));
        boolean far = !Double.isNaN(fromHeight) && Math.abs(fromHeight - s[arm]) > ARM_JUMP;
        armDisagree[hive] = far ? armDisagree[hive] + 1 : 0;
        if (far && armDisagree[hive] >= ARM_JUMP_TAGS) {
            armDisagree[hive] = 0;
            s[arm] = fromHeight;
            for (int j = 0; j < N; j++) {
                p[arm][j] = 0;
                p[j][arm] = 0;
            }
            p[arm][arm] = START_ARM_SIGMA * START_ARM_SIGMA * 4;
            armResets++;
        }

        if (!update(s, p, z, turret, id)) {
            rejected++;
            return false;
        }
        used++;

        // Odometry since the frame, again from the corrected state
        if (base != null) {
            copy(s, p, base);
            boolean replay = false;
            for (Step step : history) {
                if (replay) {
                    predict(s, p, step);
                    copy(s, p, step);
                }
                if (step == base) replay = true;
            }
        }
        System.arraycopy(s, 0, state, 0, N);
        set(cov, p);
        return true;
    }

    /** Kalman update of (s, p) with one tag; false (and nothing changed) if it's beyond the gate. */
    private boolean update(double[] s, double[][] p, double[] z, double turret, int id) {
        double[] predicted = measure(s, turret, id);
        if (predicted == null) return false;
        double[][] hm = new double[3][N];
        for (int j = 0; j < N; j++) {
            if (j >= RED_ARM && j != RED_ARM + HiveTags.hiveOf(id)) continue;
            double[] plus = s.clone(), minus = s.clone();
            double e = j < HEADING ? 1e-3 : 1e-5;
            plus[j] += e;
            minus[j] -= e;
            double[] a = measure(plus, turret, id), b = measure(minus, turret, id);
            if (a == null || b == null) return false;
            for (int i = 0; i < 3; i++) hm[i][j] = (a[i] - b[i]) / (2 * e);
        }
        // Noise: the direction to a tag is good, its distance less so (single-tag pose from four corners)
        double range = Math.sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
        double across = range * Math.tan(TAG_ANGLE_SIGMA) + TAG_POSITION_FLOOR;
        double deep = range * TAG_RANGE_FRACTION + TAG_POSITION_FLOOR;
        double[][] r = {{across * across, 0, 0}, {0, across * across, 0}, {0, 0, deep * deep}};

        double[] innovation = new double[3];
        for (int i = 0; i < 3; i++) innovation[i] = z[i] - predicted[i];
        double[][] sm = add(multiply(multiply(hm, p), transpose(hm)), r);
        double[][] si = invert3(sm);
        if (si == null) return false;
        double d2 = 0;
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) d2 += innovation[i] * si[i][j] * innovation[j];
        if (d2 > GATE) return false;

        double[][] k = multiply(multiply(p, transpose(hm)), si);
        for (int i = 0; i < N; i++) for (int j = 0; j < 3; j++) s[i] += k[i][j] * innovation[j];
        s[HEADING] = wrap(s[HEADING]);
        // Joseph form keeps the covariance symmetric and positive
        double[][] ikh = identity();
        double[][] kh = multiply(k, hm);
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) ikh[i][j] -= kh[i][j];
        double[][] next = add(multiply(multiply(ikh, p), transpose(ikh)), multiply(multiply(k, r), transpose(k)));
        set(p, next);
        return true;
    }

    /** Where the tag should be in the camera's frame from state s, or null if it's behind the lens. */
    public double[] measure(double[] s, double turret, int id) {
        double h = s[HEADING];
        double tx = s[X] + camera.turretX * Math.cos(h) - camera.turretY * Math.sin(h);
        double ty = s[Y] + camera.turretX * Math.sin(h) + camera.turretY * Math.cos(h);
        double yaw = h + turret;
        double cyaw = Math.cos(yaw), syaw = Math.sin(yaw), ct = Math.cos(camera.tilt), st = Math.sin(camera.tilt);
        double[] tag = HiveTags.position(id, s[RED_ARM + HiveTags.hiveOf(id)]);
        double dx = tag[0] - (tx + camera.ahead * cyaw);
        double dy = tag[1] - (ty + camera.ahead * syaw);
        double dz = tag[2] - camera.height;
        double forward = dx * cyaw * ct + dy * syaw * ct + dz * st;
        if (forward <= 0) return null;
        double right = dx * syaw - dy * cyaw;
        double down = dx * cyaw * st + dy * syaw * st - dz * ct;
        return new double[] {right, down, forward};
    }

    /**
     * The arm angle that puts this tag at this height above the tiles, or NaN if no angle within 60° of level does.
     * A tag's height only goes one way as its arm turns through that range, so halving the interval finds it.
     */
    static double armFromHeight(int id, double height) {
        double lo = -Math.toRadians(60), hi = Math.toRadians(60);
        double atLo = HiveTags.position(id, lo)[2] - height, atHi = HiveTags.position(id, hi)[2] - height;
        if (atLo * atHi > 0) return Double.NaN;
        for (int i = 0; i < 30; i++) {
            double mid = (lo + hi) / 2, atMid = HiveTags.position(id, mid)[2] - height;
            if (atMid * atLo > 0) {
                lo = mid;
                atLo = atMid;
            } else {
                hi = mid;
            }
        }
        return (lo + hi) / 2;
    }

    // ---- State ----

    public double x() { return state[X]; }
    public double y() { return state[Y]; }
    public double heading() { return state[HEADING]; }
    public double arm(int hive) { return state[RED_ARM + hive]; }
    /** When a tag of that hive was last in a frame (taken then, used or not), or -infinity: none yet. */
    public double lastSeen(int hive) { return lastSeen[hive]; }

    /** Standard deviation of state i (X, Y, HEADING, RED_ARM, BLUE_ARM). */
    public double sigma(int i) {
        return Math.sqrt(Math.max(0, cov[i][i]));
    }

    public int getUsed() { return used; }
    public int getRejected() { return rejected; }
    public int getArmResets() { return armResets; }

    // ---- Small matrices ----

    private static void copy(double[] s, double[][] p, Step step) {
        System.arraycopy(s, 0, step.state, 0, N);
        set(step.cov, p);
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double[][] identity() {
        double[][] m = new double[N][N];
        for (int i = 0; i < N; i++) m[i][i] = 1;
        return m;
    }

    private static void set(double[][] to, double[][] from) {
        for (int i = 0; i < to.length; i++) System.arraycopy(from[i], 0, to[i], 0, to[i].length);
    }

    private static double[][] multiply(double[][] a, double[][] b) {
        double[][] m = new double[a.length][b[0].length];
        for (int i = 0; i < a.length; i++) {
            for (int k = 0; k < b.length; k++) {
                if (a[i][k] == 0) continue;
                for (int j = 0; j < b[0].length; j++) m[i][j] += a[i][k] * b[k][j];
            }
        }
        return m;
    }

    private static double[][] transpose(double[][] a) {
        double[][] m = new double[a[0].length][a.length];
        for (int i = 0; i < a.length; i++) for (int j = 0; j < a[0].length; j++) m[j][i] = a[i][j];
        return m;
    }

    private static double[][] add(double[][] a, double[][] b) {
        double[][] m = new double[a.length][a[0].length];
        for (int i = 0; i < a.length; i++) for (int j = 0; j < a[0].length; j++) m[i][j] = a[i][j] + b[i][j];
        return m;
    }

    private static double[][] invert3(double[][] m) {
        double a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1],
                i = m[2][2];
        double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        if (Math.abs(det) < 1e-12) return null;
        return new double[][] {
                {(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det},
                {(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det},
                {(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det}};
    }
}
