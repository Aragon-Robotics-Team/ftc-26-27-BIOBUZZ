package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.vision.CameraLens;
import org.horizon36596.simloop.vision.SimFrame;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Wiffle holes on the balls in a rendered camera frame, as on the real ones (indoor-pickleball style, from the Arducam
 * ground-truth frames): HOLES round holes spread evenly over the ball, each HOLE_FRACTION of its diameter across,
 * showing the darker inside of the ball. SimLoop draws plain spheres, so the holes are painted on afterwards, only on
 * the pixels that show that ball (SimFrame's piece ids), so nothing in front of a ball gets holes. Each ball starts
 * turned its own way (seeded by its id) and turns with its spin, so the holes roll with it.
 *
 * Frame: the game pieces' field frame (FieldFrame), inches. The camera is given as SimLoop's CameraMount numbers.
 */
final class WiffleHoles {
    static final int HOLES = 26;
    static final double HOLE_FRACTION = 0.19;
    // How dark a hole is: the ball's color times this at its middle, rising toward EDGE at its rim
    private static final double MIDDLE = 0.5, EDGE = 0.75;
    private static final double[][] DIRECTIONS = evenlySpread(HOLES);

    private final CameraLens lens;
    private final double forward, left, up, yaw, pitch;
    private final Map<Integer, double[]> turned = new HashMap<>(); // per ball: rotation matrix, row-major

    /** The camera: its lens, and where it sits on the robot (CameraMount's numbers: inches, radians, pitch down). */
    WiffleHoles(CameraLens lens, double forward, double left, double up, double yaw, double pitch) {
        this.lens = lens;
        this.forward = forward;
        this.left = left;
        this.up = up;
        this.yaw = yaw;
        this.pitch = pitch;
    }

    /**
     * Turns each ball by its spin over dt seconds, then paints the holes facing the camera into rgb (the frame's bytes,
     * RGB, row by row), for a robot at (x, y) facing heading, field frame.
     */
    void paint(SimFrame frame, byte[] rgb, List<GamePiece> pieces, double x, double y, double heading, double dt) {
        double cx = x + Math.cos(heading) * forward - Math.sin(heading) * left;
        double cy = y + Math.sin(heading) * forward + Math.cos(heading) * left;
        double look = heading + yaw;
        double[] camera = {cx, cy, up};
        double[] ahead = {Math.cos(look) * Math.cos(pitch), Math.sin(look) * Math.cos(pitch), -Math.sin(pitch)};
        double[] right = {Math.sin(look), -Math.cos(look), 0};
        double[] down = cross(ahead, right);
        for (GamePiece piece : pieces) {
            double[] r = turn(piece, dt);
            if (piece.isHeld() || frame.pixelCountOfPiece(piece.getId()) == 0) continue;
            double radius = piece.getRadiusInches();
            double[] center = {piece.getFieldXInches(), piece.getFieldYInches(), piece.getFieldZInches()};
            for (double[] d0 : DIRECTIONS) {
                double[] d = times(r, d0);
                double[] hole = {center[0] + radius * d[0], center[1] + radius * d[1], center[2] + radius * d[2]};
                if (dot(minus(hole, camera), d) >= 0) continue; // faces away
                double[] t1 = cross(d, Math.abs(d[2]) < 0.9 ? new double[] {0, 0, 1} : new double[] {1, 0, 0});
                normalize(t1);
                double[] t2 = cross(d, t1);
                double size = HOLE_FRACTION * radius;
                double[] p = project(hole, camera, ahead, right, down);
                double[] p1 = project(plus(hole, t1, size), camera, ahead, right, down);
                double[] p2 = project(plus(hole, t2, size), camera, ahead, right, down);
                if (p == null || p1 == null || p2 == null) continue;
                fill(frame, rgb, piece.getId(), p, p1[0] - p[0], p1[1] - p[1], p2[0] - p[0], p2[1] - p[1]);
            }
        }
    }

    /** The ball's rotation, turned by its spin over dt; a ball seen for the first time starts at a seeded random one. */
    private double[] turn(GamePiece piece, double dt) {
        double[] r = turned.get(piece.getId());
        if (r == null) {
            Random random = new Random(piece.getId() * 7919L + 26);
            double[] axis = {random.nextGaussian(), random.nextGaussian(), random.nextGaussian()};
            r = rotation(axis, random.nextDouble() * 2 * Math.PI);
            turned.put(piece.getId(), r);
        }
        double[] spin = {piece.getSpinXRadPerSecond(), piece.getSpinYRadPerSecond(), piece.getSpinZRadPerSecond()};
        double rate = Math.sqrt(dot(spin, spin));
        if (rate * dt > 1e-9) {
            double[] next = multiply(rotation(spin, rate * dt), r);
            System.arraycopy(next, 0, r, 0, 9);
        }
        return r;
    }

    /** Darkens the pixels of piece id inside the ellipse at p with axes a and b (pixels). */
    private static void fill(SimFrame frame, byte[] rgb, int id, double[] p, double ax, double ay, double bx, double by) {
        double det = ax * by - ay * bx;
        if (Math.abs(det) < 0.25) return; // edge on: too thin to see
        int w = frame.getWidthPx(), h = frame.getHeightPx();
        double reach = Math.hypot(ax, ay) + Math.hypot(bx, by);
        int x0 = Math.max(0, (int) Math.floor(p[0] - reach)), x1 = Math.min(w - 1, (int) Math.ceil(p[0] + reach));
        int y0 = Math.max(0, (int) Math.floor(p[1] - reach)), y1 = Math.min(h - 1, (int) Math.ceil(p[1] + reach));
        for (int py = y0; py <= y1; py++) {
            for (int px = x0; px <= x1; px++) {
                double dx = px + 0.5 - p[0], dy = py + 0.5 - p[1];
                double s = (dx * by - dy * bx) / det, t = (ax * dy - ay * dx) / det;
                double q = s * s + t * t;
                if (q > 1 || frame.getPieceIdAt(px, py) != id) continue;
                double f = MIDDLE + (EDGE - MIDDLE) * q;
                int i = (py * w + px) * 3;
                for (int c = 0; c < 3; c++) rgb[i + c] = (byte) Math.round((rgb[i + c] & 0xFF) * f);
            }
        }
    }

    private double[] project(double[] point, double[] camera, double[] ahead, double[] right, double[] down) {
        double[] v = minus(point, camera);
        return lens.project(dot(v, right), dot(v, down), dot(v, ahead));
    }

    /** n directions spread evenly over the sphere (a Fibonacci lattice). */
    private static double[][] evenlySpread(int n) {
        double[][] d = new double[n][];
        double golden = Math.PI * (3 - Math.sqrt(5));
        for (int i = 0; i < n; i++) {
            double z = 1 - 2 * (i + 0.5) / n, r = Math.sqrt(1 - z * z);
            d[i] = new double[] {r * Math.cos(golden * i), r * Math.sin(golden * i), z};
        }
        return d;
    }

    /** The rotation by angle about axis (any length), row-major (Rodrigues). */
    private static double[] rotation(double[] axis, double angle) {
        double n = Math.sqrt(dot(axis, axis));
        double x = axis[0] / n, y = axis[1] / n, z = axis[2] / n, c = Math.cos(angle), s = Math.sin(angle), k = 1 - c;
        return new double[] {
                c + x * x * k, x * y * k - z * s, x * z * k + y * s,
                y * x * k + z * s, c + y * y * k, y * z * k - x * s,
                z * x * k - y * s, z * y * k + x * s, c + z * z * k};
    }

    private static double[] multiply(double[] a, double[] b) {
        double[] m = new double[9];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                m[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
            }
        }
        return m;
    }

    private static double[] times(double[] m, double[] v) {
        return new double[] {m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
                m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] minus(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double[] plus(double[] a, double[] b, double scale) {
        return new double[] {a[0] + b[0] * scale, a[1] + b[1] * scale, a[2] + b[2] * scale};
    }

    private static void normalize(double[] v) {
        double n = Math.sqrt(dot(v, v));
        for (int i = 0; i < 3; i++) v[i] /= n;
    }
}
