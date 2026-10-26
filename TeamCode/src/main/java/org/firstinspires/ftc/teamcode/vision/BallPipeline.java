package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;

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
 */
public class BallPipeline {
    private final BallGeometry geometry;

    public BallPipeline(BallGeometry geometry) {
        this.geometry = geometry;
    }

    public List<Ball> detect(Mat bgr) {
        List<Ball> balls = new ArrayList<>();
        // TODO: port ball_pipeline.py. Until then the camera finds nothing and the chase ends right away.
        return balls;
    }
}
