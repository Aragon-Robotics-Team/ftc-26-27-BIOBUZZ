package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.*;

import com.bylazar.camerastream.PanelsCameraStream;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.firstinspires.ftc.teamcode.vision.BallGeometry;
import org.firstinspires.ftc.teamcode.vision.BallPipeline;
import org.firstinspires.ftc.teamcode.vision.BallVisionProcessor;
import org.firstinspires.ftc.teamcode.vision.PollenTracker;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * The webcam by the intake looking for balls on the tiles. The {@link BallPipeline} finds them on the camera's thread;
 * each loop this turns the newest frame into field positions (with the robot pose from when the frame was taken) and
 * keeps the pollen to chase in a {@link PollenTracker}. Robot.chasePollen drives to it.
 *
 * Off unless enabled: only OpModes that chase open the camera. If the webcam isn't in the configuration, everything
 * else still runs and telemetry says so.
 */
public class BallVision implements Subsystem {
    private static final double POSE_HISTORY_S = 1;

    private final WebcamName webcam; // null if it isn't configured
    private final Supplier<Pose> pose;
    private final BallGeometry geometry;
    private final BallPipeline pipeline;
    private final BallVisionProcessor processor;
    private final List<double[]> skipped = new ArrayList<>();
    private final PollenTracker tracker;
    private VisionPortal portal;
    // A camera that runs the processor itself instead of through a VisionPortal: the sim's webcam (BallChaseSimTest)
    private boolean framesFromCamera = false;
    private boolean preview = false;

    // Robot poses from recent loops ({seconds, x, y, heading}), to place a frame by when it was taken
    private final ArrayDeque<double[]> poses = new ArrayDeque<>();
    private BallVisionProcessor.Result lastResult;
    private List<Ball> lastBalls = Collections.emptyList();
    private double lastFrameTime = Double.NaN;
    private int frames = 0;
    private double appliedExposure = Double.NaN;
    private int appliedGain = -1;

    public BallVision(HardwareMap hardwareMap, Supplier<Pose> pose) {
        this.pose = pose;
        WebcamName found;
        try {
            found = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        } catch (RuntimeException e) {
            found = null;
        }
        webcam = found;
        geometry = geometry();
        pipeline = new BallPipeline(geometry, pipelineSettings());
        processor = new BallVisionProcessor(geometry, pipeline, DISTORTION);

        PollenTracker.Settings settings = new PollenTracker.Settings();
        settings.lockGate = LOCK_GATE;
        settings.smoothing = SMOOTHING;
        settings.switchMargin = SWITCH_MARGIN;
        settings.switchFrames = SWITCH_FRAMES;
        settings.intakeReach = INTAKE_REACH;
        settings.intakeHalfWidth = INTAKE_WIDTH / 2;
        settings.intakeAxisLeft = 0;
        settings.nearMouth = NEAR_MOUTH;
        settings.edgeMarginPx = EDGE_MARGIN_PX;
        tracker = new PollenTracker(settings, this::chaseable);
    }

    /** The ball pipeline's settings from RobotConstants.BallVision; the rest keep BallPipeline's defaults. */
    public static BallPipeline.Settings pipelineSettings() {
        BallPipeline.Settings s = new BallPipeline.Settings();
        s.pollenHue = POLLEN_HUE;
        s.redNectarHue = RED_NECTAR_HUE;
        s.blueNectarHue = BLUE_NECTAR_HUE;
        s.fieldContrastK = FIELD_CONTRAST_K;
        s.fieldContrastMin = FIELD_CONTRAST_MIN;
        s.fieldContrastMax = FIELD_CONTRAST_MAX;
        s.sizeMarginLo = SIZE_MARGIN_LO;
        s.sizeMarginHi = SIZE_MARGIN_HI;
        s.minCircularity = MIN_CIRCULARITY;
        s.minCircleFill = MIN_CIRCLE_FILL;
        s.minRoundFrac = MIN_ROUND_FRAC;
        s.roundTol = ROUND_TOL;
        s.lineFilter = LINE_FILTER;
        return s;
    }

    /** The camera model from RobotConstants.BallVision: calibrated if there's a calibration. */
    public static BallGeometry geometry() {
        BallGeometry.Mount mount = new BallGeometry.Mount(LENS_FORWARD, LENS_LEFT, LENS_HEIGHT, YAW, TILT, ROLL);
        if (CAMERA_MATRIX == null) return BallGeometry.fromFov(FRAME_WIDTH, FRAME_HEIGHT, HORIZONTAL_FOV, mount);
        return BallGeometry.fromCalibration(FRAME_WIDTH, FRAME_HEIGHT, CALIBRATION_WIDTH, CALIBRATION_HEIGHT,
                CAMERA_MATRIX[0], CAMERA_MATRIX[1], CAMERA_MATRIX[2], CAMERA_MATRIX[3], mount);
    }

    /** Pollen within SKIP_RADIUS of here isn't chased for the next seconds (or until clearSkips): the robot couldn't get it. */
    public void skip(double x, double y, double seconds) {
        skipped.add(new double[] {x, y, now() + seconds});
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    public void clearSkips() {
        skipped.clear();
    }

    /** Inside the walls, off the hive's feet, away from the flowers and not skipped. */
    private boolean chaseable(double x, double y) {
        double now = now();
        for (double[] s : skipped) {
            if (now < s[2] && Math.hypot(x - s[0], y - s[1]) < SKIP_RADIUS) return false;
        }
        return onOpenFloor(x, y);
    }

    /** Inside the walls, off the hive's feet and away from the flowers. */
    static boolean onOpenFloor(double x, double y) {
        if (x < 0 || y < 0 || x > 144 || y > 144) return false;
        double m = RobotConstants.Field.KEEP_OUT_MARGIN;
        for (double[] r : RobotConstants.Field.HIVE_FEET) {
            if (x > r[0] - m && x < r[2] + m && y > r[1] - m && y < r[3] + m) return false;
        }
        for (Pose flower : RobotConstants.Field.FLOWERS) {
            if (Math.hypot(x - flower.x(), y - flower.y()) < RobotConstants.Field.FLOWER_KEEP_OUT) return false;
        }
        return true;
    }

    /**
     * Opens the camera (call during init: it takes a moment). With preview, frames are also drawn for the Driver
     * Station and the Panels camera stream, which costs CPU.
     */
    @SuppressWarnings("unchecked")
    public void enable(boolean preview) {
        if (webcam == null || portal != null || framesFromCamera) return;
        if (webcam instanceof Consumer) {
            ((Consumer<VisionProcessor>) webcam).accept(processor);
            framesFromCamera = true;
            return;
        }
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new android.util.Size(CAPTURE_WIDTH, CAPTURE_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .enableLiveView(preview)
                .build();
        this.preview = preview;
        processor.setStreaming(preview);
        if (preview) PanelsCameraStream.INSTANCE.startStream(processor, null);
    }

    public boolean isAvailable() {
        return webcam != null;
    }

    public PollenTracker tracker() {
        return tracker;
    }

    public BallGeometry getGeometry() {
        return geometry;
    }

    public BallPipeline getPipeline() {
        return pipeline;
    }

    /** New frames picked up so far. */
    public int getFrameCount() {
        return frames;
    }

    /** Balls in the newest frame. */
    public List<Ball> getBalls() {
        return lastBalls;
    }

    /** How long the pipeline took on the newest frame, milliseconds; NaN before the first. */
    public double getPipelineMs() {
        return lastResult == null ? Double.NaN : lastResult.detectMs;
    }

    /** Capture time of the newest frame, seconds (System.nanoTime), or NaN before the first. */
    public double getFrameTime() {
        return lastFrameTime;
    }

    // ---- Loop ----

    @Override
    public void update() {
        if (portal == null && !framesFromCamera) return;
        double now = System.nanoTime() / 1e9;
        Pose p = pose.get();
        poses.addLast(new double[]{now, p.x(), p.y(), p.heading()});
        while (poses.size() > 2 && poses.peekFirst()[0] < now - POSE_HISTORY_S) poses.removeFirst();

        if (portal != null && portal.getCameraState() == VisionPortal.CameraState.STREAMING) applyCameraSettings();

        BallVisionProcessor.Result result = processor.latest();
        if (result == null || result == lastResult) return;
        lastResult = result;
        frames++;
        // The camera's capture time, unless it's implausible (then when the pipeline got the frame)
        long nanos = result.captureNanos;
        if (nanos > result.processedNanos || nanos < result.processedNanos - 500_000_000L) nanos = result.processedNanos;
        lastFrameTime = nanos / 1e9;
        lastBalls = result.balls;
        tracker.update(lastFrameTime, poseAt(lastFrameTime), result.balls, geometry);

        PollenTracker.Target target = tracker.target();
        Ball ball = target == null || target.lastSeen != lastFrameTime ? null : target.lastBall;
        processor.setHighlight(ball == null ? null : new double[]{ball.x, ball.y, ball.radius});
    }

    /** The recorded pose closest in time. */
    private double[] poseAt(double time) {
        double[] best = poses.peekLast();
        for (double[] sample : poses) {
            if (Math.abs(sample[0] - time) < Math.abs(best[0] - time)) best = sample;
        }
        return new double[]{best[1], best[2], best[3]};
    }

    /** Before the match: the camera's exposure and gain set once it's streaming, so the first frames after START use them. */
    public void prepare() {
        if (portal != null && portal.getCameraState() == VisionPortal.CameraState.STREAMING) applyCameraSettings();
    }

    /** Manual exposure and gain from RobotConstants, again whenever they change. */
    private void applyCameraSettings() {
        if (EXPOSURE_MS == appliedExposure && GAIN == appliedGain) return;
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (exposure == null) return;
        if (EXPOSURE_MS > 0) {
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure((long) (EXPOSURE_MS * 1000), TimeUnit.MICROSECONDS);
            if (gain != null) gain.setGain(Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), GAIN)));
        } else {
            exposure.setMode(ExposureControl.Mode.Auto);
        }
        appliedExposure = EXPOSURE_MS;
        appliedGain = GAIN;
    }

    @Override
    public void stop() {
        if (portal == null) return;
        if (preview) PanelsCameraStream.INSTANCE.stopStream();
        portal.close();
        portal = null;
    }

    // ---- State ----

    /** One phrase for telemetry: what the camera is doing. */
    public String status() {
        if (webcam == null) return "no webcam (\"" + WEBCAM_NAME + "\" not in the configuration)";
        if (portal == null && !framesFromCamera) return "off";
        if (processor.problem() != null) return processor.problem();
        if (framesFromCamera) return String.format("sim camera, pipeline %.0f ms", lastResult == null ? 0 : lastResult.detectMs);
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return "starting (" + portal.getCameraState() + ")";
        return String.format("%.0f fps, pipeline %.0f ms", portal.getFps(), lastResult == null ? 0 : lastResult.detectMs);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Ball vision", status());
        if (portal == null && !framesFromCamera) return;
        int pollen = 0;
        for (Ball ball : lastBalls) if (ball.type == Ball.Type.POLLEN) pollen++;
        telemetry.addData("Balls", "%d (%d pollen, %d chaseable)", lastBalls.size(), pollen, tracker.chaseable().size());
        PollenTracker.Target target = tracker.target();
        if (target != null) {
            telemetry.addData("Target", "x %.1f  y %.1f, seen %.1f s ago%s", target.x, target.y,
                    System.nanoTime() / 1e9 - target.lastSeen, target.atIntake ? ", at the intake" : "");
        }
    }
}
