package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.MAX_DEPTH;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.MIN_FILL;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.POLLEN_HUE_MAX;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.POLLEN_HUE_MIN;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.POLLEN_SAT_MIN;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.POLLEN_VAL_MIN;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.SIZE_GATE_MAX;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.SIZE_GATE_MIN;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
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
 * TEMPORARY: until ball_pipeline.py is ported, {@link #detect} is a plain yellow threshold that finds POLLEN only (no
 * nectar, no lighting correction, touching balls come out as one blob). Its numbers are untuned guesses in
 * RobotConstants.BallVision ("TEMPORARY yellow-only detector"). Replace the whole method with the port.
 */
public class BallPipeline {
    private final BallGeometry geometry;

    private final Mat hsv = new Mat(), mask = new Mat(), hierarchy = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    public BallPipeline(BallGeometry geometry) {
        this.geometry = geometry;
    }

    /** TEMPORARY yellow-only detector; see the class comment. */
    public List<Ball> detect(Mat bgr) {
        List<Ball> balls = new ArrayList<>();

        // 1. Yellow pixels
        Imgproc.cvtColor(bgr, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, new Scalar(POLLEN_HUE_MIN, POLLEN_SAT_MIN, POLLEN_VAL_MIN),
                new Scalar(POLLEN_HUE_MAX, 255, 255), mask);

        // 2. Nothing above the farthest row a pollen on the tiles can be in (the horizon, or MAX_DEPTH)
        double diameter = Ball.Type.POLLEN.diameter;
        int firstRow = (int) Math.max(0, Math.min(mask.rows(), Math.floor(geometry.firstFloorRow(diameter, MAX_DEPTH))));
        if (firstRow > 0) mask.rowRange(0, firstRow).setTo(new Scalar(0));

        // 3. Drop speckle, fill small holes (glare)
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        // 4. One circle per round blob of about the right size for where it is
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 20) {
                contour.release();
                continue;
            }
            Point center = new Point();
            float[] radius = new float[1];
            MatOfPoint2f points = new MatOfPoint2f(contour.toArray());
            Imgproc.minEnclosingCircle(points, center, radius);
            points.release();
            contour.release();

            double r = radius[0];
            boolean truncated = center.x - r < 1 || center.y - r < 1
                    || center.x + r > bgr.cols() - 1 || center.y + r > bgr.rows() - 1;
            // A ball cut by the frame edge fills less of its circle, so only whole ones are held to MIN_FILL
            if (!truncated && area / (Math.PI * r * r) < MIN_FILL) continue;

            double expected = geometry.expectedDiameterPx(center.x, center.y, diameter);
            if (Double.isNaN(expected)) continue;
            double ratio = 2 * r / expected;
            if (ratio < SIZE_GATE_MIN || ratio > SIZE_GATE_MAX) continue;

            balls.add(new Ball(Ball.Type.POLLEN, center.x, center.y, r, truncated, false));
        }
        return balls;
    }
}
