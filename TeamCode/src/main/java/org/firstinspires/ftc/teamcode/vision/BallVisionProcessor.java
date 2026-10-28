package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Runs the {@link BallPipeline} on every webcam frame, on the camera's thread. The frame is turned into undistorted BGR
 * first, as the pipeline expects. The newest result is picked up by BallVision from the OpMode loop.
 *
 * Also draws what it found: on the Driver Station / RC preview, and, when {@link #setStreaming} is on, on its own
 * stream for Panels (a {@link CameraStreamSource}). Drawing costs CPU, so the chase leaves streaming off.
 */
public class BallVisionProcessor implements VisionProcessor, CameraStreamSource {
    /** What one frame held. */
    public static class Result {
        public final List<Ball> balls;
        public final long captureNanos;   // System.nanoTime() when the camera took the frame
        public final long processedNanos; // System.nanoTime() when the pipeline started on it
        public final double detectMs;

        Result(List<Ball> balls, long captureNanos, long processedNanos, double detectMs) {
            this.balls = balls;
            this.captureNanos = captureNanos;
            this.processedNanos = processedNanos;
            this.detectMs = detectMs;
        }
    }

    private final BallGeometry geometry;
    private final BallPipeline pipeline;
    private final double[] distortion; // null: frames are used as they come

    private final Mat small = new Mat(), bgr = new Mat(), undistorted = new Mat(), annotated = new Mat();
    private Mat mapX, mapY;
    private double scale = 1; // camera frame pixels per pipeline pixel
    private volatile Result latest;
    private volatile String problem;
    private volatile double[] highlight;  // u, v, r of the ball being chased
    private volatile boolean streaming = false;
    private final AtomicReference<Bitmap> streamFrame = new AtomicReference<>();

    public BallVisionProcessor(BallGeometry geometry, BallPipeline pipeline, double[] distortion) {
        this.geometry = geometry;
        this.pipeline = pipeline;
        this.distortion = distortion;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // A bigger frame of the same aspect is shrunk to the pipeline's size
        scale = width / (double) geometry.width;
        if (Math.abs(height / (double) geometry.height - scale) > 0.01 * scale) {
            problem = String.format("camera sends %dx%d, not a resize of %dx%d", width, height, geometry.width,
                    geometry.height);
            return;
        }
        if (distortion != null) {
            Mat k = new Mat(3, 3, CvType.CV_64F);
            k.put(0, 0, geometry.fx, 0, geometry.cx, 0, geometry.fy, geometry.cy, 0, 0, 1);
            Mat d = new Mat(1, distortion.length, CvType.CV_64F);
            d.put(0, 0, distortion);
            mapX = new Mat();
            mapY = new Mat();
            Calib3d.initUndistortRectifyMap(k, d, new Mat(), k, new Size(geometry.width, geometry.height),
                    CvType.CV_16SC2, mapX, mapY);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (problem != null) return null;
        long start = System.nanoTime();
        Mat src = frame;
        if (frame.cols() != geometry.width) {
            Imgproc.resize(frame, small, new Size(geometry.width, geometry.height), 0, 0, Imgproc.INTER_AREA);
            src = small;
        }
        Imgproc.cvtColor(src, bgr, src.channels() == 4 ? Imgproc.COLOR_RGBA2BGR : Imgproc.COLOR_RGB2BGR);
        Mat input = bgr;
        if (mapX != null) {
            Imgproc.remap(bgr, undistorted, mapX, mapY, Imgproc.INTER_LINEAR);
            input = undistorted;
        }
        List<Ball> balls = Collections.unmodifiableList(pipeline.detect(input));
        Result result = new Result(balls, captureTimeNanos, start, (System.nanoTime() - start) / 1e6);
        latest = result;
        if (streaming) stream(input, result);
        return result;
    }

    /** The newest frame's result, or null before the first one. */
    public Result latest() {
        return latest;
    }

    /** Why nothing is being detected, or null. */
    public String problem() {
        return problem;
    }

    /** Ring this ball (pixels: u, v, r) in the drawings; null for none. */
    public void setHighlight(double[] ball) {
        highlight = ball;
    }

    /** Draw each frame for {@link #getFrameBitmap} (Panels camera stream). */
    public void setStreaming(boolean streaming) {
        this.streaming = streaming;
    }

    // ---- Drawing ----

    private static int color(Ball.Type type) {
        switch (type) {
            case POLLEN: return Color.rgb(255, 214, 10);
            case RED_NECTAR: return Color.rgb(255, 60, 60);
            default: return Color.rgb(40, 140, 255);
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {
        if (!(userContext instanceof Result)) return;
        // Balls are in pipeline pixels; the canvas scale is for camera frame pixels
        scaleBmpPxToCanvasPx *= (float) scale;
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        paint.setAntiAlias(true);
        paint.setStrokeWidth(3 * scaleCanvasDensity);
        for (Ball ball : ((Result) userContext).balls) {
            paint.setColor(color(ball.type));
            canvas.drawCircle((float) ball.x * scaleBmpPxToCanvasPx, (float) ball.y * scaleBmpPxToCanvasPx,
                    (float) ball.radius * scaleBmpPxToCanvasPx, paint);
        }
        double[] h = highlight;
        if (h != null) {
            paint.setColor(Color.WHITE);
            paint.setStrokeWidth(6 * scaleCanvasDensity);
            canvas.drawCircle((float) h[0] * scaleBmpPxToCanvasPx, (float) h[1] * scaleBmpPxToCanvasPx,
                    (float) (h[2] + 6) * scaleBmpPxToCanvasPx, paint);
        }
    }

    private void stream(Mat frame, Result result) {
        Imgproc.cvtColor(frame, annotated, Imgproc.COLOR_BGR2RGBA);
        for (Ball ball : result.balls) {
            int c = color(ball.type);
            Imgproc.circle(annotated, new Point(ball.x, ball.y), (int) Math.round(ball.radius),
                    new Scalar(Color.red(c), Color.green(c), Color.blue(c), 255), 2);
        }
        double[] h = highlight;
        if (h != null) {
            Imgproc.circle(annotated, new Point(h[0], h[1]), (int) Math.round(h[2] + 4), new Scalar(255, 255, 255, 255), 3);
        }
        Bitmap bitmap = Bitmap.createBitmap(annotated.cols(), annotated.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(annotated, bitmap);
        streamFrame.set(bitmap);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        Bitmap bitmap = streamFrame.get(); // null before the first frame: the stream skips it
        continuation.dispatch(consumer -> consumer.accept(bitmap));
    }
}
