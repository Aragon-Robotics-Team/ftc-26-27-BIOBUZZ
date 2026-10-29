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
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * The ball camera (the webcam by the intake). There's no VisionPortal in the sim, so BallVision.enable() hands this its
 * frame processor (BallVisionProcessor, the real one), and every few ticks this draws what the lens sees and runs the
 * processor on it, as the portal's camera thread would.
 *
 * Drawn through the webcam's calibration and distortion from the CAD mount, like BallVisionSimTest, with the field
 * walls, the flower columns and the hive feet as boxes that hide balls behind them. The colors are PLACEHOLDERS: flat,
 * one light, no shadows, not measured from real frames.
 */
final class SimWebcam extends AbstractFakeDevice implements WebcamName, java.util.function.Consumer<VisionProcessor> {
    // PLACEHOLDER colors, 0xRRGGBB: inside the pipeline's hue windows, but not measured from real balls or tiles
    static final int POLLEN_RGB = 0xF2C21B, RED_NECTAR_RGB = 0xC8282D, BLUE_NECTAR_RGB = 0x2D6BD2;
    static final int TILE_RGB = 0x7A7A7A, SEAM_RGB = 0x5A5A5A, WALL_RGB = 0x3A3A3A, BOX_RGB = 0x9A9A9A;
    static final int FRAME_EVERY = 2; // ticks: 25 frames a second at the sim's 50 Hz. PLACEHOLDER: the webcam's real rate

    private final SimFrameRenderer renderer;
    private final int width, height;
    private final CameraOccluders boxes;
    private VisionProcessor processor;
    private Mat rgb;
    private int ticks;
    private int frames;
    private Path saveTo;
    private int saveEvery;

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

    /** One sim tick: a frame every FRAME_EVERY ticks, once BallVision has enabled the camera. Field frame, inches. */
    void tick(GamePieceTracker field, double x, double y, double heading) {
        if (processor == null || ticks++ % FRAME_EVERY != 0) return;
        SimFrame frame = renderer.render(field.getPieces(), x, y, heading, boxes, AprilTagLayout.empty());
        if (rgb == null) rgb = new Mat(frame.getHeightPx(), frame.getWidthPx(), CvType.CV_8UC3);
        rgb.put(0, 0, frame.getRgbBytes());
        // No capture time: BallVision then uses when the pipeline got the frame, which in the sim is the same moment
        processor.processFrame(rgb, Long.MAX_VALUE);
        if (saveTo != null && frames % saveEvery == 0) save(frame, saveTo.resolve(String.format("frame-%04d.png", frames)));
        frames++;
    }

    /** Also writes every n-th frame to dir as a PNG, to look at what the camera saw. */
    void saveFrames(Path dir, int n) throws IOException {
        Files.createDirectories(dir);
        try (DirectoryStream<Path> old = Files.newDirectoryStream(dir, "frame-*.png")) {
            for (Path png : old) Files.delete(png);
        }
        saveTo = dir;
        saveEvery = n;
    }

    private static void save(SimFrame frame, Path png) {
        try (OutputStream out = Files.newOutputStream(png)) {
            frame.writePng(out);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
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
