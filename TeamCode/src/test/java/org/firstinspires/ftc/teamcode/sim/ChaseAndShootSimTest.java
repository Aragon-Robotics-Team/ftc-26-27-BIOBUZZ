package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.tuning.ChaseAndShootTest;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;

/**
 * The Chase And Shoot Test OpMode for 30 s: the pollen chase on the real ball vision (SimWebcam) until the robot has
 * 4, then the drive to the launch spot and the shots into red's up cell. Same field as BallChaseSimTest: red's left
 * corner, an empty hopper, balls scattered over the tiles (no staged nectar, so 4 pollen don't tip the hive).
 * Logged to chase-and-shoot.rlog.
 */
class ChaseAndShootSimTest {
    private static final long SEED = 26;
    private static final int POLLEN = 20, NECTAR = 8;

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void itChasesFourThenShootsThemIntoTheHive() {
        BiobuzzRobotSim sim = new BiobuzzRobotSim().preload(0).scatter(SEED, POLLEN, NECTAR);
        sim.placeAt(RobotConstants.Field.RED_LEFT_CORNER.x(), RobotConstants.Field.RED_LEFT_CORNER.y(),
                RobotConstants.Field.RED_LEFT_CORNER.heading());

        OpModeRunResult result = sim.runner(ChaseAndShootTest.class)
                .initLoopTicks(40)
                .activeTicks(1500) // 30 s
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    // Red (B) in init, after init's 25-tick Pinpoint wait; the left corner is the default
                    gamepad1.b = phase == OpModePhase.INIT_LOOP && tick == 30;
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "chase-and-shoot.rlog"));

        String stage = SimTelemetry.lastLine(result.telemetry().transmissions(), "Stage");
        String chase = SimTelemetry.lastLine(result.telemetry().transmissions(), "Chase");
        System.out.println("Chase and shoot warnings: " + result.warnings());
        System.out.printf("Chase and shoot: %s; %s; %d shots, %d still held, %d in red's up cell, robot at x %.1f y %.1f%n",
                stage, chase, sim.shotsFired(), sim.field.getHeldCount(), sim.match.cellBalls(Alliance.RED), sim.x(), sim.y());

        assertTrue(sim.shotsFired() >= 1, "it should shoot what it picked up: " + stage + "; " + chase);
    }
}
