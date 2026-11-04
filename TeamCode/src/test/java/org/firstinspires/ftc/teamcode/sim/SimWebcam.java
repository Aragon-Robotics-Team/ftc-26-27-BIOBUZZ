package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.BallVision;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.firstinspires.ftc.teamcode.vision.BallGeometry;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.horizon36596.simloop.fakehardware.AbstractFakeDevice;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.ObstacleBox;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraLens;
import org.horizon36596.simloop.vision.CameraMount;
import org.horizon36596.simloop.vision.CameraOccluders;
import org.horizon36596.simloop.vision.FrameStyle;
import org.horizon36596.simloop.vision.SimFrame;
import org.horizon36596.simloop.vision.SimFrameRenderer;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import org.horizon36596.simloop.field.GamePiece;

import java.util.ArrayList;
import java.util.List;

/**
 * The ball camera (the webcam by the intake). There's no VisionPortal in the sim, so BallVision.enable() hands this its
 * frame processor (BallVisionProcessor, the real one), and every few ticks this draws what the lens sees and runs the
 * processor on it, as the portal's camera thread would.
 *
 * Drawn through the webcam's calibration and distortion from the CAD mount, like BallVisionSimTest, with the field
 * walls, the flower columns and the hive feet as boxes that hide balls behind them, and wiffle holes painted on the
 * balls (WiffleHoles). The colors are PLACEHOLDERS: flat, one light, no shadows, not measured from real frames.
 */
final class SimWebcam extends AbstractFakeDevice implements WebcamName, java.util.function.Consumer<VisionProcessor> {
    // PLACEHOLDER colors, 0xRRGGBB: inside the pipeline's hue windows, but not measured from real balls or tiles
    static final int POLLEN_RGB = 0xF2C21B, RED_NECTAR_RGB = 0xC8282D, BLUE_NECTAR_RGB = 0x2D6BD2;
    static final int TILE_RGB = 0x7A7A7A, SEAM_RGB = 0x5A5A5A, WALL_RGB = 0x3A3A3A, BOX_RGB = 0x9A9A9A;
    static final int FRAME_EVERY = 2; // ticks: 25 frames a second at the sim's 50 Hz, unless realism() says otherwise
    private static final double TICK_S = 0.02; // the sim's tick

    private final SimFrameRenderer renderer;
    private final int width, height;
    private final CameraOccluders boxes;
    private final WiffleHoles holes;
    private VisionProcessor processor;
    private Mat rgb;
    private int ticks;
    private int frames;
    private Path saveTo;
    private int saveEvery;
    // realism(): a frame every frameEvery ticks, its result lagTicks after it's taken, each ball missing from it by
    // chance (null: none)
    private int frameEvery = FRAME_EVERY, lagTicks;
    private java.util.Random missing;
    private double missFraction;
    private final java.util.ArrayDeque<Object[]> pending = new java.util.ArrayDeque<>(); // {due tick, Mat, capture ns}

    SimWebcam() {
        BallGeometry geometry = BallVision.geometry();
        width = (int) geometry.width;
        height = (int) geometry.height;
        double[] d = RobotConstants.BallVision.DISTORTION;
        CameraLens lens = new CameraLens(geometry.fx, geometry.fy, geometry.cx, geometry.cy, geometry.width,
                geometry.height).withDistortion(d[0], d[1], d[2], d[3], d[4]);
        CameraMount mount = new CameraMount(RobotConstants.BallVision.LENS_FORWARD, RobotConstants.BallVision.LENS_LEFT,
                RobotConstants.BallVision.LENS_HEIGHT, RobotConstants.BallVision.YAW, RobotConstants.BallVision.TILT);
        renderer = SimFrameRenderer.builder(lens, mount)
                .style(FrameStyle.builder()
                        .floor(TILE_RGB, 24, SEAM_RGB, 0.5)
                        .background(WALL_RGB)
                        .boxes(BOX_RGB)
                        .pieceColour(Ball.Type.POLLEN.name(), POLLEN_RGB)
                        .pieceColour(Ball.Type.RED_NECTAR.name(), RED_NECTAR_RGB)
                        .pieceColour(Ball.Type.BLUE_NECTAR.name(), BLUE_NECTAR_RGB)
                        .build())
                .sensorNoise(36596, 2.0) // seeded; PLACEHOLDER spread, 8-bit counts
                .build();
        boxes = CameraOccluders.field(fieldBoxes());
        holes = new WiffleHoles(lens, RobotConstants.BallVision.LENS_FORWARD, RobotConstants.BallVision.LENS_LEFT,
                RobotConstants.BallVision.LENS_HEIGHT, RobotConstants.BallVision.YAW, RobotConstants.BallVision.TILT);
    }

    /** What hides balls from the camera: the walls (12 in tall), the flower columns and the hive's feet. */
    private static List<ObstacleBox> fieldBoxes() {
        List<ObstacleBox> list = new ArrayList<>();
        list.add(new ObstacleBox("wallAudience", 72, 73, -73, 73, 0, 12));
        list.add(new ObstacleBox("wallRear", -73, -72, -73, 73, 0, 12));
        list.add(new ObstacleBox("wallRed", -73, 73, -73, -72, 0, 12));
        list.add(new ObstacleBox("wallBlue", -73, 73, 72, 73, 0, 12));
        for (int i = 0; i < RobotConstants.Field.FLOWERS.length; i++) {
            double[] c = SimField.flowerCenter(i);
            list.add(new ObstacleBox("flower" + i, c[0] - 2.65, c[0] + 2.65, c[1] - 2.65, c[1] + 2.65, 0, 21.5));
        }
        for (int side = -1; side <= 1; side += 2) {
            double y = side * SimField.FRAME_HALF_WIDTH;
            list.add(new ObstacleBox("hiveFoot" + side, -SimField.FRAME_HALF_DEPTH - 1, SimField.FRAME_HALF_DEPTH + 1,
                    y - 1, y + 1, 0, 1.5));
        }
        return list;
    }

    /** BallVision.enable(): the processor this camera feeds from now on. */
    @Override
    public void accept(VisionProcessor processor) {
        this.processor = processor;
        processor.init(width, height, null);
    }

    /**
     * The camera as a Control Hub runs it rather than as fast as the sim can: a frame every frameEvery ticks, the
     * pipeline's result lagTicks later (stamped with when it was taken, as the camera stamps frames), and each ball
     * left out of a frame with chance missFraction (seeded), as the real pipeline misses balls (52 to 84% found on
     * the hand-labeled frames). Before the run.
     */
    void realism(int frameEvery, int lagTicks, long seed, double missFraction) {
        this.frameEvery = frameEvery;
        this.lagTicks = lagTicks;
        this.missing = new java.util.Random(seed);
        this.missFraction = missFraction;
    }

    /** One sim tick: a frame every frameEvery ticks, once BallVision has enabled the camera. Field frame, inches. */
    void tick(GamePieceTracker field, double x, double y, double heading) {
        if (processor == null) return;
        while (!pending.isEmpty() && (int) pending.peekFirst()[0] <= ticks) {
            Object[] due = pending.pollFirst();
            process((Mat) due[1], (long) due[2]);
            ((Mat) due[1]).release();
        }
        if (ticks++ % frameEvery != 0) return;
        List<GamePiece> pieces = field.getPieces();
        if (missing != null) {
            List<GamePiece> seen = new ArrayList<>();
            for (GamePiece piece : pieces) if (missing.nextDouble() >= missFraction) seen.add(piece);
            pieces = seen;
        }
        SimFrame frame = renderer.render(pieces, x, y, heading, boxes, AprilTagLayout.empty());
        if (rgb == null) rgb = new Mat(frame.getHeightPx(), frame.getWidthPx(), CvType.CV_8UC3);
        byte[] bytes = frame.getRgbBytes();
        holes.paint(frame, bytes, pieces, x, y, heading, frameEvery * TICK_S);
        rgb.put(0, 0, bytes);
        long taken = simNanos();
        if (lagTicks == 0) process(rgb, taken);
        else pending.addLast(new Object[] {ticks - 1 + lagTicks, rgb.clone(), taken});
    }

    /**
     * The sim clock the team code reads, nanoseconds: its System.nanoTime() is SimLoop's sim-clock hook, in the copy
     * the processor's class loader made.
     */
    private long simNanos() {
        try {
            Class<?> hooks = processor.getClass().getClassLoader()
                    .loadClass("org.horizon36596.simloop.simclock.SimClockHooks");
            return (long) hooks.getMethod("nanoTime").invoke(null);
        } catch (ReflectiveOperationException | IllegalStateException e) {
            return Long.MAX_VALUE; // no sim clock (a bare unit test): BallVision then uses when the pipeline got it
        }
    }

    private void process(Mat image, long captureNanos) {
        Object result = processor.processFrame(image, captureNanos);
        if (saveTo != null && frames % saveEvery == 0) {
            Mat shown = image.clone();
            drawFound(shown, result);
            save(shown, saveTo.resolve(String.format("frame-%04d.png", frames)));
            shown.release();
        }
        frames++;
    }

    /**
     * Rings what the pipeline found, RGB: green on pollen, white on nectar. The result is read by field name: an OpMode
     * run loads the team's classes again (SimClockLoader), so its BallVisionProcessor.Result isn't this one's.
     */
    private static void drawFound(Mat rgb, Object result) {
        if (result == null) return;
        try {
            for (Object b : (List<?>) result.getClass().getField("balls").get(result)) {
                Class<?> c = b.getClass();
                boolean pollen = c.getField("type").get(b).toString().equals(Ball.Type.POLLEN.name());
                org.opencv.core.Scalar color = pollen
                        ? new org.opencv.core.Scalar(40, 230, 60) : new org.opencv.core.Scalar(255, 255, 255);
                org.opencv.imgproc.Imgproc.circle(rgb, new org.opencv.core.Point(c.getField("x").getDouble(b),
                        c.getField("y").getDouble(b)), (int) Math.round(c.getField("radius").getDouble(b) + 3), color, 2);
            }
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("BallVisionProcessor.Result changed shape", e);
        }
    }

    /** Also writes every n-th frame to dir as a PNG, with what the pipeline found ringed, to look at what it saw. */
    void saveFrames(Path dir, int n) throws IOException {
        Files.createDirectories(dir);
        try (DirectoryStream<Path> old = Files.newDirectoryStream(dir, "frame-*.png")) {
            for (Path png : old) Files.delete(png);
        }
        saveTo = dir;
        saveEvery = n;
    }

    /** Writes the frame as the pipeline got it, holes and all, as a PNG. */
    static void save(Mat rgb, Path png) {
        Mat bgr = new Mat();
        org.opencv.imgproc.Imgproc.cvtColor(rgb, bgr, org.opencv.imgproc.Imgproc.COLOR_RGB2BGR);
        if (!org.opencv.imgcodecs.Imgcodecs.imwrite(png.toString(), bgr)) {
            throw new UncheckedIOException(new IOException("couldn't write " + png));
        }
        bgr.release();
    }

    int frames() {
        return frames;
    }

    // ---- WebcamName, as the SDK's lookup and BallVision need it ----

    @Override
    public void update(double deltaTime) {
        // Frames come from tick(), which knows where the robot is
    }

    @Override public SerialNumber getSerialNumber() { return SerialNumber.createFake(); }
    @Override public String getUsbDeviceNameIfAttached() { return null; }
    @Override public boolean isAttached() { return true; }
    @Override public boolean isWebcam() { return true; }
    @Override public boolean isCameraDirection() { return false; }
    @Override public boolean isSwitchable() { return false; }
    @Override public boolean isUnknown() { return false; }
    @Override public void asyncRequestCameraPermission(android.content.Context context, Deadline deadline,
            Continuation<? extends Consumer<Boolean>> continuation) { }
    @Override public boolean requestCameraPermission(Deadline deadline) { return true; }
    @Override public CameraCharacteristics getCameraCharacteristics() { return null; }
}
