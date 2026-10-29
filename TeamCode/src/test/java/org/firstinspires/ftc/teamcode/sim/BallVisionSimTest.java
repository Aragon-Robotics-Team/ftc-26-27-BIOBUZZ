package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.BallVision;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.firstinspires.ftc.teamcode.vision.BallGeometry;
import org.firstinspires.ftc.teamcode.vision.BallPipeline;
import org.firstinspires.ftc.teamcode.vision.BallVisionProcessor;
import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraLens;
import org.horizon36596.simloop.vision.CameraMount;
import org.horizon36596.simloop.vision.CameraOccluders;
import org.horizon36596.simloop.vision.FrameStyle;
import org.horizon36596.simloop.vision.SimFrame;
import org.horizon36596.simloop.vision.SimFrameRenderer;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/**
 * The real ball pipeline (BallVisionProcessor: resize, undistort, BallPipeline) on a rendered webcam frame. There's no
 * VisionPortal in the sim, so the test hands the processor frames itself, as the portal would.
 *
 * The frame is drawn through the webcam's real calibration and distortion (RobotConstants.BallVision) from the CAD
 * mount, so undistortion and the pixel-to-floor geometry are exercised too. The colors are PLACEHOLDERS: flat,
 * one light, no shadows. Tune them toward real frames (biobuzz_detection's captures) before trusting thresholds.
 * The frame is saved to build/sim/ball-frame.png to look at.
 */
class BallVisionSimTest {
    // PLACEHOLDER colors, 0xRRGGBB: inside the pipeline's hue windows, but not measured from real balls or tiles
    private static final int POLLEN_RGB = 0xF2C21B, RED_NECTAR_RGB = 0xC8282D, BLUE_NECTAR_RGB = 0x2D6BD2;
    private static final int TILE_RGB = 0x7A7A7A, SEAM_RGB = 0x5A5A5A, WALL_RGB = 0x3A3A3A;

    /** One ball on the tiles, robot frame (inches, +x forward, +y left). */
    private static final class Placed {
        final Ball.Type type;
        final double forward, left;
        Placed(Ball.Type type, double forward, double left) {
            this.type = type;
            this.forward = forward;
            this.left = left;
        }
    }

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally();
    }

    @Test
    void theRealPipelineFindsEachBallWhereItIs() throws IOException {
        List<Placed> placed = new ArrayList<>();
        placed.add(new Placed(Ball.Type.POLLEN, 24, 6));
        placed.add(new Placed(Ball.Type.RED_NECTAR, 36, -4));
        placed.add(new Placed(Ball.Type.BLUE_NECTAR, 30, 16));

        // The robot sits at the field origin facing +X, so field coordinates are robot coordinates
        GamePieceTracker field = new GamePieceTracker(placed.size(), FieldWorld.unbounded());
        for (Placed p : placed) field.addLoosePiece(p.type.name(), p.forward, p.left, p.type.diameter / 2);

        BallGeometry geometry = BallVision.geometry();
        double[] d = RobotConstants.BallVision.DISTORTION;
        CameraLens lens = new CameraLens(geometry.fx, geometry.fy, geometry.cx, geometry.cy, geometry.width,
                geometry.height).withDistortion(d[0], d[1], d[2], d[3], d[4]);
        CameraMount mount = new CameraMount(RobotConstants.BallVision.LENS_FORWARD, RobotConstants.BallVision.LENS_LEFT,
                RobotConstants.BallVision.LENS_HEIGHT, RobotConstants.BallVision.YAW, RobotConstants.BallVision.TILT);
        SimFrameRenderer renderer = SimFrameRenderer.builder(lens, mount)
                .style(FrameStyle.builder()
                        .floor(TILE_RGB, 24, SEAM_RGB, 0.5)
                        .background(WALL_RGB)
                        .pieceColour(Ball.Type.POLLEN.name(), POLLEN_RGB)
                        .pieceColour(Ball.Type.RED_NECTAR.name(), RED_NECTAR_RGB)
                        .pieceColour(Ball.Type.BLUE_NECTAR.name(), BLUE_NECTAR_RGB)
                        .build())
                .supersampling(2)
                .sensorNoise(36596, 2.0) // seeded; PLACEHOLDER spread, 8-bit counts
                .build();
        SimFrame frame = renderer.render(field.getPieces(), 0, 0, 0, CameraOccluders.none(), AprilTagLayout.empty());
        Path png = Paths.get("build", "sim", "ball-frame.png");
        Files.createDirectories(png.getParent());
        try (OutputStream out = Files.newOutputStream(png)) {
            frame.writePng(out);
        }

        BallVisionProcessor processor = new BallVisionProcessor(geometry,
                new BallPipeline(geometry, BallVision.pipelineSettings()), RobotConstants.BallVision.DISTORTION);
        processor.init(frame.getWidthPx(), frame.getHeightPx(), null);
        assertNull(processor.problem(), "the processor refused the frame size");
        Mat rgb = new Mat(frame.getHeightPx(), frame.getWidthPx(), CvType.CV_8UC3);
        BallVisionProcessor.Result result;
        try {
            rgb.put(0, 0, frame.getRgbBytes());
            result = (BallVisionProcessor.Result) processor.processFrame(rgb, 0);
        } finally {
            rgb.release();
        }
        assertNotNull(result);
        System.out.println("BallVision found " + result.balls + " in " + String.format("%.0f ms", result.detectMs));

        assertEquals(placed.size(), result.balls.size(), "one detection per ball: " + result.balls);
        for (Placed p : placed) {
            Ball found = null;
            for (Ball b : result.balls) if (b.type == p.type) found = b;
            assertNotNull(found, "no " + p.type + " found: " + result.balls);
            double[] at = geometry.robotPosition(found.x, found.y, p.type.diameter);
            assertNotNull(at, p.type + " at a pixel no ball on the tiles could be at");
            System.out.printf("  %s at (%.1f, %.1f), really (%.1f, %.1f)%n", p.type, at[0], at[1], p.forward, p.left);
            assertEquals(p.forward, at[0], 1.5, p.type + " forward, inches");
            assertEquals(p.left, at[1], 1.5, p.type + " left, inches");
        }
    }
}
