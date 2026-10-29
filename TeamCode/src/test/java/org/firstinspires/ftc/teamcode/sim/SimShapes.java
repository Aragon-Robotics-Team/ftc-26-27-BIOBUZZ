package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.field.ConvexShape;

import java.util.ArrayList;
import java.util.List;

/** Convex solids for SimLoop bodies, built from outlines instead of hand-wound triangles. */
final class SimShapes {
    private SimShapes() {}

    /**
     * A prism joining two faces: corner i of {@code a} to corner i of {@code b}. Each face is a convex polygon with its
     * corners in order around it, either way round. ConvexShape wants every triangle wound outward; this winds them
     * from the solid's centroid.
     */
    static ConvexShape prism(String name, double[][] a, double[][] b) {
        int n = a.length;
        double[][] v = new double[2 * n][];
        for (int i = 0; i < n; i++) {
            v[i] = a[i].clone();
            v[n + i] = b[i].clone();
        }
        List<int[]> faces = new ArrayList<>();
        for (int i = 1; i < n - 1; i++) {
            faces.add(new int[] {0, i, i + 1});
            faces.add(new int[] {n, n + i, n + i + 1});
        }
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            faces.add(new int[] {i, j, n + j});
            faces.add(new int[] {i, n + j, n + i});
        }
        double[] c = new double[3];
        for (double[] p : v) for (int k = 0; k < 3; k++) c[k] += p[k] / v.length;
        int[][] triangles = new int[faces.size()][];
        for (int f = 0; f < faces.size(); f++) {
            int[] t = faces.get(f);
            double[] p = v[t[0]], q = v[t[1]], r = v[t[2]];
            double[] u = {q[0] - p[0], q[1] - p[1], q[2] - p[2]}, w = {r[0] - p[0], r[1] - p[1], r[2] - p[2]};
            double[] normal = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]};
            double out = normal[0] * (p[0] - c[0]) + normal[1] * (p[1] - c[1]) + normal[2] * (p[2] - c[2]);
            triangles[f] = out >= 0 ? t : new int[] {t[0], t[2], t[1]};
        }
        return new ConvexShape(name, v, triangles);
    }

    /** A prism along x from x0 to x1 over a convex (y, z) outline. */
    static ConvexShape alongX(String name, double x0, double x1, double[][] yz) {
        double[][] a = new double[yz.length][], b = new double[yz.length][];
        for (int i = 0; i < yz.length; i++) {
            a[i] = new double[] {x0, yz[i][0], yz[i][1]};
            b[i] = new double[] {x1, yz[i][0], yz[i][1]};
        }
        return prism(name, a, b);
    }

    /**
     * A vertical ring around (cx, cy) from z0 to z1, as {@code segments} straight pieces (a ring isn't convex, so
     * it's several solids): inner radius r, wall thickness t.
     */
    static List<ConvexShape> ring(String name, double cx, double cy, double r, double t, double z0, double z1,
                                  int segments) {
        List<ConvexShape> pieces = new ArrayList<>();
        double step = 2 * Math.PI / segments;
        for (int i = 0; i < segments; i++) {
            double a0 = i * step, a1 = a0 + step;
            double[][] outline = {
                    {cx + r * Math.cos(a0), cy + r * Math.sin(a0)}, {cx + (r + t) * Math.cos(a0), cy + (r + t) * Math.sin(a0)},
                    {cx + (r + t) * Math.cos(a1), cy + (r + t) * Math.sin(a1)}, {cx + r * Math.cos(a1), cy + r * Math.sin(a1)}};
            double[][] bottom = new double[4][], top = new double[4][];
            for (int k = 0; k < 4; k++) {
                bottom[k] = new double[] {outline[k][0], outline[k][1], z0};
                top[k] = new double[] {outline[k][0], outline[k][1], z1};
            }
            pieces.add(prism(name + "_" + i, bottom, top));
        }
        return pieces;
    }
}
