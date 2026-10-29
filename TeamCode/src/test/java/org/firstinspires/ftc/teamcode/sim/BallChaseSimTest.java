package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.tuning.BallChaseTest;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Paths;

/**
 * The Ball Chase Test OpMode for 30 s, on the real ball vision: the sim webcam (SimWebcam) draws what the camera by
 * the intake sees, BallVision's pipeline finds the balls in it, PollenTracker picks the target and PollenChase drives
 * the intake over it, turning in place to look when there's nothing to chase. The robot starts in red's left corner
 * with an empty hopper, on a field with balls scattered over the tiles (SimField.scatter, seeded) instead of the
 * match's staging: pollen to chase and nectar it should leave alone.
 *
 * Logged to ball-chase.rlog; one camera frame a second goes to build/sim/ball-chase-frames/.
 */
class BallChaseSimTest {
    private static final long SEED = 26; // another seed, another field
    private static final int POLLEN = 20, NECTAR = 8;

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void theChasePicksUpPollenItSees() throws IOException {
        BiobuzzRobotSim sim = new BiobuzzRobotSim().preload(0).scatter(SEED, POLLEN, NECTAR);
        double startX = RobotConstants.Field.RED_LEFT_CORNER.x(), startY = RobotConstants.Field.RED_LEFT_CORNER.y();
        sim.placeAt(startX, startY, RobotConstants.Field.RED_LEFT_CORNER.heading());
        sim.webcam.saveFrames(Paths.get("build", "sim", "ball-chase-frames"), 25);

        OpModeRunResult result = sim.runner(BallChaseTest.class)
                .initLoopTicks(40)
                .activeTicks(1500) // 30 s
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    // Red (B) in init, after init's 25-tick Pinpoint wait; the left corner is the default
                    gamepad1.b = phase == OpModePhase.INIT_LOOP && tick == 30;
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "ball-chase.rlog"));

        int picked = 0, nectarHeld = 0;
        for (GamePiece piece : sim.field.getPieces()) {
            if (!piece.isHeld()) continue;
            if (piece.getKind().equals(Ball.Type.POLLEN.name())) picked++;
            else nectarHeld++;
        }
        String chase = SimTelemetry.lastLine(result.telemetry().transmissions(), "Chase");
        System.out.println("Ball chase warnings: " + result.warnings());
        System.out.printf("Ball chase: %d frames, holding %d of %d pollen and %d nectar; robot at x %.1f y %.1f; %s%n",
                sim.webcam.frames(), picked, POLLEN, nectarHeld, sim.x(), sim.y(), chase);
        for (double[] ball : sim.scattered()) {
            System.out.printf("  %s at x %.0f y %.0f%n", ball[0] == 0 ? "pollen" : ball[0] == 1 ? "red nectar" : "blue nectar",
                    ball[1], ball[2]);
        }

        assertTrue(sim.webcam.frames() > 0, "BallVision never enabled the camera");
        assertTrue(picked >= 1, "the chase should pick up some of the scattered pollen: " + chase);
        assertTrue(nectarHeld == 0, "the intake can't take nectar, but holds " + nectarHeld);
    }
}
