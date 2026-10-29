package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.opmodes.TeleOp;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.List;

/** The real TeleOp on the simulated robot. */
class TeleOpSimTest {
    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void fullStickDrivesTheRobotAndThePinpointFollows() {
        BiobuzzRobotSim sim = new BiobuzzRobotSim();
        sim.placeAt(36, 24, 0); // open floor ahead: from the field center it would drive into the hive's foot
        double startX = sim.x(), startY = sim.y(), startHeading = sim.heading();

        OpModeRunResult result = sim.runner(TeleOp.class)
                .initLoopTicks(30)
                .activeTicks(100) // 2 s
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    // Full forward for 1 s, then let go: Pedro's pose is checked once the robot has stopped, since
                    // the last telemetry is sent before the final tick's motion
                    gamepad1.left_stick_y = phase == OpModePhase.ACTIVE && tick < 50 ? -1 : 0;
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "teleop.rlog")); // open in AdvantageScope
        System.out.println("TeleOp warnings: " + result.warnings());

        double moved = Math.hypot(sim.x() - startX, sim.y() - startY);
        System.out.printf("TeleOp moved %.1f in%n", moved);
        assertTrue(moved > 20, "one second of full stick should cover well over 20 in, moved " + moved);
        assertEquals(startHeading, sim.heading(), Math.toRadians(5), "driving straight shouldn't turn the robot");

        // TeleOp starts Pedro at (0, 0), so compare how far each thinks the robot went
        List<List<String>> sent = result.telemetry().transmissions();
        double[] pedro = SimTelemetry.lastPose(sent);
        assertEquals(moved, Math.hypot(pedro[0], pedro[1]), 1.0, "Pedro's Pinpoint pose should match the real motion");
    }
}
