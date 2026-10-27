package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Finds the balls in one camera frame: the Java port of biobuzz_detection's ball_pipeline.py (LightingResistance),
 * stages 1-10. Choosing which ball to chase is not part of it (see {@link PollenTracker} and PollenChase).
 *
 * - The frame is 8-bit BGR (CV_8UC3), already undistorted, at 640x400. Don't modify or keep it.
 * - The camera's size and position come from {@link #geometry} (row-size gate, horizon).
 * - Report every ball of every {@link Ball.Type}; which ones to chase is decided elsewhere.
 * - It may keep state between frames; one instance is used for the camera stream.
 *
 * Stages, as in the Python reference (its README explains each one; tune there, against the ground-truth set):
 *  1. Light blur, then field-referenced color: the field (gray tiles, white walls, black wall base) is neutral, so its
 *     color in the frame is the light's tint. Per-channel gains that make it gray again are estimated every frame.
 *  2. Color masks: broad hue windows per type, plus a field-contrast check (saturation above fieldContrastK x the
 *     frame's median) and an adaptive value floor. Rows above the horizon are cleared.
 *  3-4. Ball-colored pixels to gray, heavier blur.
 *  5-6. Canny + Hough: straight segments longer than most of a ball (wall bases, seams, tape) are erased from the
 *     masks. A segment with color on both sides is a chord through a ball and is kept.
 *  7. Contours per type; piles of touching balls are cut apart with a watershed seeded at ball-radius scale.
 *  8. Size gate: a ball on the tiles has a known pixel size at each spot of the frame (camera geometry).
 *  9. Circle gates on the convex hull, with two fallbacks: a circle fitted to the visible arc of a ball cut off by the
 *     frame edge (truncated), and the largest round part of a ball merged with same-color structure (recovered).
 *  10. A circle centered inside a bigger one of the same type (a wiffle hole) is dropped.
 */
public class BallPipeline {
    /** ball_pipeline.py's CONFIG; the defaults are its values. The robot's come from RobotConstants.BallVision. */
    public static class Settings {
        // Hue windows, OpenCV H (0-179), judged on the field-referenced frame. Red wraps around 0, so it has two.
        public int[][] pollenHue = {{12, 45}};
        public int[][] redNectarHue = {{0, 10}, {165, 179}};
        public int[][] blueNectarHue = {{88, 132}};

        // Field-referenced color
        public double fieldContrastK = 4.5;       // saturation cutoff: this x the frame's median saturation...
        public double fieldContrastMin = 75;      // ...clamped to this range
        public double fieldContrastMax = 130;
        public double fieldGainMin = 0.5;         // sanity limit on the per-channel gains
        public double fieldGainMax = 2.0;
        public double fieldGainSmoothing = 0.3;   // weight of each new frame's gains
        public double valueMin = 35;              // value floor: 0.6 x the 20th percentile of V, clamped to this range
        public double valueMax = 95;
        public double valueSmoothing = 0.25;

        // Size gate: tolerance around the expected pixel diameter, and the range (inches from the lens) a ball counts in
        public double sizeMarginLo = 0.55;
        public double sizeMarginHi = 1.60;
        public double minRange = 8;
        public double maxRange = 200;

        public int blur1 = 5;                     // kernel sizes (odd): color segmentation, then edges
        public int blur2 = 7;

        // Straight-line rejection
        public boolean lineFilter = true;         // off saves the most time on a slow CPU
        public double cannyLo = 40;
        public double cannyHi = 120;
        public int houghThreshold = 30;
        public double houghMaxGap = 6;
        public int lineSidePx = 4;                // a segment with color this far to both sides is inside a blob

        // Splitting touching balls: seeds where the distance to the blob's edge reaches these fractions of the expected
        // radius; each blob keeps the level that gives the most seeds
        public boolean watershedSplit = true;
        public double[] splitSeedLevels = {0.55, 0.70, 0.85};

        // Circle gates, on the convex hull (wiffle holes bite into the outline, not the hull)
        public double minCircularity = 0.65;      // 4 pi area / perimeter^2
        public double minAspect = 0.70;           // bounding box width / height
        public double maxAspect = 1.42;
        public int minCorners = 6;                // approxPolyDP vertices; boxes give 4
        public double minCircleFill = 0.60;       // hull area / enclosing circle area
        public double minSolidity = 0.45;         // contour area / hull area

        // Balls cut off by the frame edge: a circle fitted to the visible arc
        public boolean allowTruncated = true;
        public int borderMarginPx = 3;
        public double maxFitResidual = 0.12;      // RMS fit error, x radius
        public double maxStraightFrac = 1.2;      // longest straight hull edge, x radius
        public int minArcPoints = 6;              // hull points off the border needed to fit

        // Balls merged with same-color structure (nectar on alliance tape): the blob's edge must sit at the same
        // distance (within roundTol) from its deepest point in at least minRoundFrac of directions
        public boolean recoverAttached = true;
        public double minRoundFrac = 0.75;
        public double roundTol = 0.05;
        public double recoverSizeLo = 0.75;
    }

    private static final Ball.Type[] TYPES = Ball.Type.values();
    private static final double TINY_AREA = 20;  // px^2: specks aren't even considered

    private final BallGeometry geometry;
    private final Settings settings;
    private final int width, height;

    // Fixed by the geometry: expected ball radius (px) at every pixel per type, +inf where a ball can't be centered
    private final Mat[] radiusMap = new Mat[TYPES.length];
    private final Mat[] firstSeedMap = new Mat[TYPES.length]; // radiusMap x the lowest seed level
    private final double[] seedLevels;
    private final int firstRow;       // rows above it can't hold any ball
    private final double minLineLength;

    // Lighting estimates, smoothed across frames
    private double[] gains;
    private double valueFloor = Double.NaN;

    private final Mat blurred = new Mat(), balanced = new Mat(), hsv = new Mat(), small = new Mat();
    private final Mat channel = new Mat(), bright = new Mat(), hist = new Mat();
    private final Mat gray = new Mat(), foreground = new Mat(), grayBlur = new Mat(), edges = new Mat(), lines = new Mat();
    private final Mat union = new Mat(), filled = new Mat(), dist = new Mat(), cmp = new Mat(), part = new Mat();
    private final Mat[] masks = new Mat[TYPES.length];
    private final Mat kernel3 = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
    private final Mat kernel5 = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    public BallPipeline(BallGeometry geometry) {
        this(geometry, new Settings());
    }

    public BallPipeline(BallGeometry geometry, Settings settings) {
        this.geometry = geometry;
        this.settings = settings;
        width = geometry.width;
        height = geometry.height;
        seedLevels = settings.splitSeedLevels.clone();
        Arrays.sort(seedLevels);

        int first = height;
        double longest = 0;
        for (Ball.Type type : TYPES) {
            int t = type.ordinal();
            float[] r = new float[width * height];
            for (int v = 0; v < height; v++) {
                for (int u = 0; u < width; u++) {
                    double d = expectedDiameter(u, v, type.diameter);
                    r[v * width + u] = Double.isNaN(d) ? Float.POSITIVE_INFINITY : (float) (d / 2);
                    if (!Double.isNaN(d) && v < first) first = v;
                }
            }
            radiusMap[t] = new Mat(height, width, CvType.CV_32F);
            radiusMap[t].put(0, 0, r);
            firstSeedMap[t] = new Mat();
            Core.multiply(radiusMap[t], new Scalar(seedLevels[0]), firstSeedMap[t]);
            longest = Math.max(longest, referenceDiameter(type.diameter));
            masks[t] = new Mat();
        }
        // A little margin keeps the top of a far ball whose center is just below the horizon
        firstRow = Math.max(0, first - 10);
        minLineLength = Math.max(12, (int) (0.6 * longest));
    }

    public List<Ball> detect(Mat bgr) {
        // Stage 1: blur, then referenced to the field's color
        Imgproc.GaussianBlur(bgr, blurred, new Size(settings.blur1, settings.blur1), 0);
        double[] g = updateGains(blurred);
        Core.multiply(blurred, new Scalar(g[0], g[1], g[2]), balanced);
        Imgproc.cvtColor(balanced, hsv, Imgproc.COLOR_BGR2HSV);

        // Stage 2: color masks. Saturation has to stand out from the field's; value has to clear the shadow floor
        int sat = (int) fieldContrastThreshold();
        int val = (int) updateValueFloor();
        union.create(height, width, CvType.CV_8U);
        union.setTo(new Scalar(0));
        for (Ball.Type type : TYPES) {
            Mat m = masks[type.ordinal()];
            hueMask(hues(type), sat, val, m);
            if (firstRow > 0) m.rowRange(0, Math.min(firstRow, height)).setTo(new Scalar(0));
            Imgproc.morphologyEx(m, m, Imgproc.MORPH_OPEN, kernel3);
            Imgproc.morphologyEx(m, m, Imgproc.MORPH_CLOSE, kernel5);
            Core.bitwise_or(union, m, union);
        }

        // Stages 3-6: ball-colored pixels as gray, blur, edges; straight segments are erased from the masks
        if (settings.lineFilter) {
            Imgproc.cvtColor(bgr, gray, Imgproc.COLOR_BGR2GRAY);
            foreground.create(height, width, CvType.CV_8U);
            foreground.setTo(new Scalar(0));
            gray.copyTo(foreground, union);
            Imgproc.GaussianBlur(foreground, grayBlur, new Size(settings.blur2, settings.blur2), 0);
            Imgproc.Canny(grayBlur, edges, settings.cannyLo, settings.cannyHi);
            removeStraightLines();
        }

        // Stages 7-10, per type: each has its own diameter, and a pollen in front of a nectar is still two balls
        List<Ball> balls = new ArrayList<>();
        for (Ball.Type type : TYPES) {
            Mat mask = masks[type.ordinal()];
            List<MatOfPoint> contours = settings.watershedSplit ? splitTouchingBalls(mask, type) : externalContours(mask);
            List<Ball> found = new ArrayList<>();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < TINY_AREA) continue;
                Ball ball = classify(contour, area, type);
                if (ball == null && settings.recoverAttached) ball = recoverAttached(contour, type);
                if (ball != null) found.add(ball);
            }
            release(contours);
            balls.addAll(suppressInnerCircles(found));
        }
        return balls;
    }

    // ---- Geometry ----

    /**
     * Pixel diameter of a ball of this diameter centered at pixel (u, v), the way ball_pipeline.py has it (its gates
     * were tuned with it): fy x diameter / range, with the range along the ray in the un-rolled image's vertical
     * plane. NaN if no ball on the tiles can be there, or it's outside minRange-maxRange. The size gate allows
     * sizeMarginLo-sizeMarginHi x this.
     */
    public double expectedDiameter(double u, double v, double diameter) {
        double depth = geometry.depth(u, v, diameter);
        if (Double.isNaN(depth)) return Double.NaN;
        double a = (u - geometry.cx) / geometry.fx, b = (v - geometry.cy) / geometry.fy;
        double level = b * Math.cos(geometry.mount.roll) - a * Math.sin(geometry.mount.roll);
        double range = depth * Math.sqrt(1 + level * level);
        if (range < settings.minRange || range > settings.maxRange) return Double.NaN;
        return geometry.fy * diameter / range;
    }

    /** {min, max} pixel diameter for a ball centered at (u, v), or null if one can't be there. */
    private double[] sizeBounds(double u, double v, double diameter) {
        double d = expectedDiameter(u, v, diameter);
        if (Double.isNaN(d)) return null;
        return new double[]{d * settings.sizeMarginLo, d * settings.sizeMarginHi};
    }

    /** A typical diameter low in the frame, to scale the line filter. */
    private double referenceDiameter(double diameter) {
        for (double frac : new double[]{0.75, 0.85, 0.95}) {
            double d = expectedDiameter(geometry.cx, height * frac, diameter);
            if (!Double.isNaN(d)) return d;
        }
        return geometry.fy * diameter / settings.minRange;
    }

    // ---- Stages 1-2: color ----

    /**
     * BGR gains that make the field gray. Works in log chromaticity (log R/G, log B/G) on a small copy of the frame,
     * skipping clipped and near-black pixels, then keeps the half nearest the estimate a few times, so balls and
     * colorful clutter don't pull it.
     */
    private double[] updateGains(Mat frame) {
        Imgproc.resize(frame, small, new Size(80, 50), 0, 0, Imgproc.INTER_AREA);
        byte[] px = new byte[(int) small.total() * 3];
        small.get(0, 0, px);
        int n = 0;
        double[] lr = new double[px.length / 3], lb = new double[px.length / 3];
        for (int i = 0; i < px.length; i += 3) {
            int b = px[i] & 0xFF, g = px[i + 1] & 0xFF, r = px[i + 2] & 0xFF;
            int max = Math.max(b, Math.max(g, r));
            if (max <= 40 || max >= 250) continue;
            lr[n] = Math.log((r + 1.0) / (g + 1.0));
            lb[n] = Math.log((b + 1.0) / (g + 1.0));
            n++;
        }
        double[] fresh = {1, 1, 1};
        if (n >= 100) {
            double mr = median(lr.clone(), n), mb = median(lb.clone(), n);
            double[] d = new double[n], keptR = new double[n], keptB = new double[n];
            for (int iteration = 0; iteration < 3; iteration++) {
                for (int i = 0; i < n; i++) d[i] = Math.hypot(lr[i] - mr, lb[i] - mb);
                double cut = median(d.clone(), n);
                int k = 0;
                for (int i = 0; i < n; i++) {
                    if (d[i] > cut) continue;
                    keptR[k] = lr[i];
                    keptB[k] = lb[i];
                    k++;
                }
                mr = median(keptR, k);
                mb = median(keptB, k);
            }
            fresh[0] = clamp(Math.exp(-mb), settings.fieldGainMin, settings.fieldGainMax);
            fresh[2] = clamp(Math.exp(-mr), settings.fieldGainMin, settings.fieldGainMax);
        }
        if (gains == null) {
            gains = fresh;
        } else {
            for (int c = 0; c < 3; c++) gains[c] += settings.fieldGainSmoothing * (fresh[c] - gains[c]);
        }
        return gains;
    }

    /** Saturation a pixel needs to stand out from the field: a multiple of the median over non-dark pixels. */
    private double fieldContrastThreshold() {
        Core.extractChannel(hsv, channel, 2);
        Imgproc.threshold(channel, bright, 40, 255, Imgproc.THRESH_BINARY);
        Core.extractChannel(hsv, channel, 1);
        int median = percentile(histogram(channel, bright), 0.5);
        if (median < 0) return settings.fieldContrastMax;
        return clamp(settings.fieldContrastK * median, settings.fieldContrastMin, settings.fieldContrastMax);
    }

    /** Value floor that rejects shadows without cutting dimly lit balls: 0.6 x the 20th percentile of V, smoothed. */
    private double updateValueFloor() {
        Core.extractChannel(hsv, channel, 2);
        double fresh = clamp(percentile(histogram(channel, new Mat()), 0.2) * 0.6, settings.valueMin, settings.valueMax);
        valueFloor = Double.isNaN(valueFloor) ? fresh : valueFloor + settings.valueSmoothing * (fresh - valueFloor);
        return valueFloor;
    }

    private double[] histogram(Mat image, Mat mask) {
        Imgproc.calcHist(Collections.singletonList(image), new MatOfInt(0), mask, hist, new MatOfInt(256),
                new MatOfFloat(0, 256));
        float[] counts = new float[256];
        hist.get(0, 0, counts);
        double[] out = new double[256];
        for (int i = 0; i < 256; i++) out[i] = counts[i];
        return out;
    }

    /** First bin where the cumulative count reaches this fraction of the total, or -1 for an empty histogram. */
    private static int percentile(double[] counts, double fraction) {
        double total = 0;
        for (double c : counts) total += c;
        if (total == 0) return -1;
        double sum = 0;
        for (int i = 0; i < counts.length; i++) {
            sum += counts[i];
            if (sum >= fraction * total) return i;
        }
        return counts.length - 1;
    }

    private int[][] hues(Ball.Type type) {
        switch (type) {
            case POLLEN: return settings.pollenHue;
            case RED_NECTAR: return settings.redNectarHue;
            default: return settings.blueNectarHue;
        }
    }

    private void hueMask(int[][] windows, int sat, int val, Mat out) {
        out.create(height, width, CvType.CV_8U);
        out.setTo(new Scalar(0));
        for (int[] w : windows) {
            Core.inRange(hsv, new Scalar(w[0], sat, val), new Scalar(w[1], 255, 255), part);
            Core.bitwise_or(out, part, out);
        }
    }

    // ---- Stages 5-6: straight lines ----

    private void removeStraightLines() {
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, settings.houghThreshold, minLineLength, settings.houghMaxGap);
        if (lines.empty()) return;
        int[] seg = new int[(int) lines.total() * 4];
        lines.get(0, 0, seg);

        // The interior test looks at the masks with holes filled
        List<MatOfPoint> outer = externalContours(union);
        filled.create(height, width, CvType.CV_8U);
        filled.setTo(new Scalar(0));
        Imgproc.drawContours(filled, outer, -1, new Scalar(255), -1);
        release(outer);
        byte[] fill = new byte[width * height];
        filled.get(0, 0, fill);

        int off = settings.lineSidePx;
        for (int i = 0; i < seg.length; i += 4) {
            double x1 = seg[i], y1 = seg[i + 1], dx = seg[i + 2] - x1, dy = seg[i + 3] - y1;
            double len = Math.max(Math.hypot(dx, dy), 1e-6);
            double nx = -dy / len, ny = dx / len;
            int both = 0;
            for (int k = 0; k < 9; k++) {
                double t = 0.1 + 0.1 * k, px = x1 + t * dx, py = y1 + t * dy;
                if (filledAt(fill, px + off * nx, py + off * ny) && filledAt(fill, px - off * nx, py - off * ny)) both++;
            }
            if (both / 9.0 >= 0.7) continue; // a chord through a blob
            Point p1 = new Point(seg[i], seg[i + 1]), p2 = new Point(seg[i + 2], seg[i + 3]);
            for (Mat m : masks) Imgproc.line(m, p1, p2, new Scalar(0), 3);
        }
    }

    private boolean filledAt(byte[] fill, double x, double y) {
        int u = (int) clamp(Math.rint(x), 0, width - 1), v = (int) clamp(Math.rint(y), 0, height - 1);
        return fill[v * width + u] != 0;
    }

    // ---- Stage 7: contours, splitting touching balls ----

    private static List<MatOfPoint> externalContours(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        return contours;
    }

    /**
     * Contours of the mask with piles of touching balls cut apart. The distance to a blob's edge peaks near each ball's
     * center at about the ball's radius, which the geometry knows everywhere, so each spot over a fraction of it seeds
     * one ball and a watershed cuts the blob at the necks. Blobs with fewer than two seeds keep their own contour (the
     * watershed would round off a box's corners).
     */
    private List<MatOfPoint> splitTouchingBalls(Mat mask, Ball.Type type) {
        List<MatOfPoint> outer = externalContours(mask);
        // Wiffle holes filled, so they don't cap the distance below the seed level
        filled.create(height, width, CvType.CV_8U);
        filled.setTo(new Scalar(0));
        Imgproc.drawContours(filled, outer, -1, new Scalar(255), -1);
        Imgproc.distanceTransform(filled, dist, Imgproc.DIST_L2, 3);
        Core.compare(dist, firstSeedMap[type.ordinal()], cmp, Core.CMP_GE);
        if (Core.countNonZero(cmp) == 0) return outer; // no blob can hold a seed

        List<MatOfPoint> contours = new ArrayList<>();
        for (MatOfPoint contour : outer) {
            Rect r = Imgproc.boundingRect(contour);
            Mat radius = radiusMap[type.ordinal()].submat(r);
            Mat blob = Mat.zeros(r.height, r.width, CvType.CV_8U);
            Imgproc.drawContours(blob, Collections.singletonList(contour), -1, new Scalar(1), -1, Imgproc.LINE_8,
                    new Mat(), Integer.MAX_VALUE, new Point(-r.x, -r.y));
            Mat d = Mat.zeros(r.height, r.width, CvType.CV_32F);
            dist.submat(r).copyTo(d, blob); // the box may overlap other blobs

            // The seed level with the most seeds (the lowest on ties)
            int best = 1;
            Mat bestLabels = null;
            if (Core.minMaxLoc(d).maxVal >= seedLevels[0] * Core.minMaxLoc(radius).minVal) {
                Mat threshold = new Mat(), seeds = new Mat();
                for (double level : seedLevels) {
                    Core.multiply(radius, new Scalar(level), threshold);
                    Core.compare(d, threshold, seeds, Core.CMP_GE);
                    Mat labels = new Mat();
                    int n = Imgproc.connectedComponents(seeds, labels) - 1;
                    if (n > best) {
                        best = n;
                        if (bestLabels != null) bestLabels.release();
                        bestLabels = labels;
                    } else {
                        labels.release();
                    }
                }
                threshold.release();
                seeds.release();
            }
            if (bestLabels == null) {
                contours.add(contour);
            } else {
                contours.addAll(watershed(r, blob, bestLabels, best));
                contour.release();
                bestLabels.release();
            }
            radius.release();
            blob.release();
            d.release();
        }
        return contours;
    }

    /** The blob in box r cut into one piece per seed label (1..seeds), by a watershed on the blurred frame. */
    private List<MatOfPoint> watershed(Rect r, Mat blob, Mat labels, int seeds) {
        int pad = 2;
        int x0 = Math.max(0, r.x - pad), y0 = Math.max(0, r.y - pad);
        int x1 = Math.min(width, r.x + r.width + pad), y1 = Math.min(height, r.y + r.height + pad);
        Mat markers = new Mat(y1 - y0, x1 - x0, CvType.CV_32S, new Scalar(1)); // 1 = background
        Mat region = markers.submat(r.y - y0, r.y - y0 + r.height, r.x - x0, r.x - x0 + r.width);
        region.setTo(new Scalar(0), blob);
        Mat seedMask = new Mat(), seedValues = new Mat();
        Core.compare(labels, new Scalar(0), seedMask, Core.CMP_GT);
        Core.add(labels, new Scalar(1), seedValues);
        seedValues.copyTo(region, seedMask);
        Mat image = blurred.submat(y0, y1, x0, x1);
        Imgproc.watershed(image, markers);

        List<MatOfPoint> pieces = new ArrayList<>();
        Mat piece = new Mat();
        for (int label = 1; label <= seeds; label++) {
            Core.compare(region, new Scalar(label + 1), piece, Core.CMP_EQ);
            Core.bitwise_and(piece, blob, piece);
            List<MatOfPoint> found = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(piece, found, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE,
                    new Point(r.x, r.y));
            hierarchy.release();
            MatOfPoint largest = null;
            double largestArea = -1;
            for (MatOfPoint c : found) {
                double area = Imgproc.contourArea(c);
                if (area > largestArea) {
                    largest = c;
                    largestArea = area;
                }
            }
            for (MatOfPoint c : found) if (c != largest) c.release();
            if (largest != null) pieces.add(largest);
        }
        piece.release();
        seedMask.release();
        seedValues.release();
        region.release();
        image.release();
        markers.release();
        return pieces;
    }

    // ---- Stages 8-9: size and shape ----

    /** The ball this contour is, or null. */
    private Ball classify(MatOfPoint contour, double area, Ball.Type type) {
        Rect r = Imgproc.boundingRect(contour);
        int m = settings.borderMarginPx;
        Point[] hull = hull(contour);
        if (settings.allowTruncated && (r.x <= m || r.y <= m || r.x + r.width >= width - m
                || r.y + r.height >= height - m)) {
            return classifyTruncated(hull, type);
        }

        double[] bounds = sizeBounds(r.x + r.width / 2.0, r.y + r.height / 2.0, type.diameter);
        if (bounds == null) return null; // at or above the horizon
        double lo = bounds[0], hi = bounds[1];
        if (Math.max(r.width, r.height) < lo || Math.max(r.width, r.height) > hi) return null;
        if (area < 0.5 * Math.PI * lo * lo / 4 || area > Math.PI * hi * hi / 4) return null;
        double aspect = r.width / (double) r.height;
        if (aspect < settings.minAspect || aspect > settings.maxAspect) return null;

        MatOfPoint2f hull2f = new MatOfPoint2f(hull);
        try {
            double hullArea = Imgproc.contourArea(hull2f);
            if (hullArea <= 0 || area / hullArea < settings.minSolidity) return null;
            double perimeter = Imgproc.arcLength(hull2f, true);
            if (perimeter <= 0) return null;
            if (4 * Math.PI * hullArea / (perimeter * perimeter) < settings.minCircularity) return null;
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(hull2f, approx, 0.02 * perimeter, true);
            long corners = approx.total();
            approx.release();
            if (corners < settings.minCorners) return null;
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(hull2f, center, radius);
            if (radius[0] < lo / 2 || radius[0] > hi / 2) return null;
            if (hullArea / (Math.PI * radius[0] * radius[0]) < settings.minCircleFill) return null;
            return new Ball(type, center.x, center.y, radius[0]);
        } finally {
            hull2f.release();
        }
    }

    /**
     * A contour touching the frame edge: a circle fitted to the hull points off the border. The fit has to be tight,
     * with no long straight hull edge (a box), ball-sized at the fitted center, and mostly filled where it's in frame.
     */
    private Ball classifyTruncated(Point[] hull, Ball.Type type) {
        List<Point> arc = new ArrayList<>();
        for (Point p : hull) if (offBorder(p)) arc.add(p);
        if (arc.size() < settings.minArcPoints) return null;
        double[] circle = fitCircle(arc);
        if (circle == null) return null;
        double cx = circle[0], cy = circle[1], r = circle[2];
        if (circle[3] > settings.maxFitResidual * r) return null;
        for (int i = 0; i < hull.length; i++) {
            Point a = hull[i], b = hull[(i + 1) % hull.length];
            if (offBorder(a) && offBorder(b) && Math.hypot(b.x - a.x, b.y - a.y) > settings.maxStraightFrac * r) {
                return null;
            }
        }
        double[] bounds = sizeBounds(cx, cy, type.diameter);
        if (bounds == null || 2 * r < bounds[0] || 2 * r > bounds[1]) return null;

        // The hull against the in-frame part of the circle
        int x0 = (int) Math.max(0, cx - r), y0 = (int) Math.max(0, cy - r);
        int x1 = (int) Math.min(width, cx + r + 1), y1 = (int) Math.min(height, cy + r + 1);
        if (x1 <= x0 || y1 <= y0) return null;
        Mat canvas = Mat.zeros(y1 - y0, x1 - x0, CvType.CV_8U);
        Imgproc.circle(canvas, new Point((int) (cx - x0), (int) (cy - y0)), (int) r, new Scalar(255), -1);
        int visible = Core.countNonZero(canvas);
        canvas.release();
        MatOfPoint2f hull2f = new MatOfPoint2f(hull);
        double hullArea = Imgproc.contourArea(hull2f);
        hull2f.release();
        if (visible <= 0 || hullArea / visible < settings.minCircleFill) return null;
        return new Ball(type, cx, cy, r, true, false);
    }

    private boolean offBorder(Point p) {
        int m = settings.borderMarginPx;
        return p.x > m && p.x < width - 1 - m && p.y > m && p.y < height - 1 - m;
    }

    private static Point[] hull(MatOfPoint contour) {
        MatOfInt indices = new MatOfInt();
        Imgproc.convexHull(contour, indices);
        Point[] points = contour.toArray();
        int[] idx = indices.toArray();
        indices.release();
        Point[] hull = new Point[idx.length];
        for (int i = 0; i < idx.length; i++) hull[i] = points[idx[i]];
        return hull;
    }

    /** Least-squares (Kasa) circle through the points: {cx, cy, r, rms residual}, or null if they're on a line. */
    private static double[] fitCircle(List<Point> points) {
        double mx = 0, my = 0;
        for (Point p : points) {
            mx += p.x;
            my += p.y;
        }
        mx /= points.size();
        my /= points.size();
        // u^2 + v^2 = 2a u + 2b v + c around the mean; normal equations
        double suu = 0, svv = 0, suv = 0, su = 0, sv = 0, sz = 0, szu = 0, szv = 0, n = points.size();
        for (Point p : points) {
            double u = p.x - mx, v = p.y - my, z = u * u + v * v;
            suu += u * u;
            svv += v * v;
            suv += u * v;
            su += u;
            sv += v;
            sz += z;
            szu += z * u;
            szv += z * v;
        }
        double[][] m = {{4 * suu, 4 * suv, 2 * su}, {4 * suv, 4 * svv, 2 * sv}, {2 * su, 2 * sv, n}};
        double[] rhs = {2 * szu, 2 * szv, sz};
        double det = det3(m);
        if (Math.abs(det) < 1e-9) return null;
        double[] x = new double[3];
        for (int col = 0; col < 3; col++) {
            double[][] mc = {m[0].clone(), m[1].clone(), m[2].clone()};
            for (int row = 0; row < 3; row++) mc[row][col] = rhs[row];
            x[col] = det3(mc) / det;
        }
        double r = Math.sqrt(Math.max(x[2] + x[0] * x[0] + x[1] * x[1], 1e-9));
        double cx = x[0] + mx, cy = x[1] + my, sq = 0;
        for (Point p : points) {
            double e = Math.hypot(p.x - cx, p.y - cy) - r;
            sq += e * e;
        }
        return new double[]{cx, cy, r, Math.sqrt(sq / n)};
    }

    private static double det3(double[][] m) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    /**
     * A ball merged with same-color structure (nectar on alliance tape, a cell edge), for a contour that failed the
     * gates. The center is the blob's deepest point. From there the blob's edge has to be about the same distance away
     * in most directions (a ball with tape attached is round except toward the tape; a box's edge distance swings),
     * and that distance has to be ball-sized for where it is.
     */
    private Ball recoverAttached(MatOfPoint contour, Ball.Type type) {
        Rect box = Imgproc.boundingRect(contour);
        Mat radius = radiusMap[type.ordinal()].submat(box);
        boolean possible = Core.minMaxLoc(radius).minVal < Double.POSITIVE_INFINITY;
        radius.release();
        if (!possible) return null; // nowhere in it can a ball sit

        int pad = 4, bw = box.width + 2 * pad, bh = box.height + 2 * pad;
        Mat blob = Mat.zeros(bh, bw, CvType.CV_8U);
        Imgproc.drawContours(blob, Collections.singletonList(contour), -1, new Scalar(255), -1, Imgproc.LINE_8,
                new Mat(), Integer.MAX_VALUE, new Point(pad - box.x, pad - box.y));
        Mat blobDist = new Mat();
        Imgproc.distanceTransform(blob, blobDist, Imgproc.DIST_L2, 3);
        Core.MinMaxLocResult peak = Core.minMaxLoc(blobDist);
        blobDist.release();
        byte[] on = new byte[bw * bh];
        blob.get(0, 0, on);
        blob.release();

        double r = peak.maxVal + 0.5; // to the edge pixel's center
        double cx = peak.maxLoc.x + box.x - pad, cy = peak.maxLoc.y + box.y - pad;
        double expected = expectedDiameter(cx, cy, type.diameter);
        if (Double.isNaN(expected) || 2 * r < settings.recoverSizeLo * expected
                || 2 * r > settings.sizeMarginHi * expected) {
            return null;
        }

        // March out from the center in 72 directions to where the blob ends (skipping ends outside the frame)
        int steps = (int) Math.ceil((2 * r + 1) / 0.5);
        double[] edge = new double[72];
        int n = 0;
        for (int i = 0; i < 72; i++) {
            double angle = 2 * Math.PI * i / 72, cos = Math.cos(angle), sin = Math.sin(angle);
            for (int k = 0; k < steps; k++) {
                double step = 0.5 * r + 0.5 * k;
                int gx = (int) Math.rint(cx + cos * step), gy = (int) Math.rint(cy + sin * step);
                int bx = gx - box.x + pad, by = gy - box.y + pad;
                if (bx >= 0 && bx < bw && by >= 0 && by < bh && on[by * bw + bx] != 0) continue;
                if (gx >= 0 && gx < width && gy >= 0 && gy < height) edge[n++] = step;
                break;
            }
        }
        if (n < 0.6 * 72) return null;
        double median = median(Arrays.copyOf(edge, n), n);
        int round = 0;
        for (int i = 0; i < n; i++) if (Math.abs(edge[i] - median) <= settings.roundTol * median + 0.5) round++;
        if (round / (double) n < settings.minRoundFrac) return null;
        if (2 * median < settings.recoverSizeLo * expected || 2 * median > settings.sizeMarginHi * expected) return null;
        return new Ball(type, cx, cy, median, false, true);
    }

    // ---- Stage 10 ----

    /** Biggest first; a ball whose center is inside one already kept is a wiffle hole. */
    private static List<Ball> suppressInnerCircles(List<Ball> balls) {
        List<Ball> sorted = new ArrayList<>(balls);
        Collections.sort(sorted, (a, b) -> Double.compare(b.radius, a.radius));
        List<Ball> kept = new ArrayList<>();
        for (Ball ball : sorted) {
            boolean inside = false;
            for (Ball k : kept) {
                if (Math.hypot(ball.x - k.x, ball.y - k.y) < k.radius) {
                    inside = true;
                    break;
                }
            }
            if (!inside) kept.add(ball);
        }
        return kept;
    }

    // ---- Helpers ----

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    /** Median of the first n values (sorts them in place). */
    private static double median(double[] values, int n) {
        Arrays.sort(values, 0, n);
        return n % 2 == 1 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
    }

    private static void release(List<? extends Mat> mats) {
        for (Mat m : mats) m.release();
    }
}
