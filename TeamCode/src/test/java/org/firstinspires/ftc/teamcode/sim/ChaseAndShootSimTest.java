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
 * The Chase And Shoot Test OpMode for 2 minutes: cycles of the pollen chase on the real ball vision (SimWebcam) until
 * the robot has 4, the drive to the launch spot and the shots into the cell hive vision says is up. Same field as
 * BallChaseSimTest: red's left corner, an empty hopper, balls scattered over the tiles and no nectar staged in the
 * cells, so the second 4 pollen tip red's hive and later shots have to follow it to the other cell.
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
    void itKeepsChasingAndShootingIntoWhicheverCellIsUp() {
        BiobuzzRobotSim sim = new BiobuzzRobotSim().preload(0).scatter(SEED, POLLEN, NECTAR);
        sim.placeAt(RobotConstants.Field.RED_LEFT_CORNER.x(), RobotConstants.Field.RED_LEFT_CORNER.y(),
                RobotConstants.Field.RED_LEFT_CORNER.heading());

        OpModeRunResult result = sim.runner(ChaseAndShootTest.class)
                .initLoopTicks(40)
                .activeTicks(6000) // 2 minutes
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    // Red (B) in init, after init's 25-tick Pinpoint wait; the left corner is the default
                    gamepad1.b = phase == OpModePhase.INIT_LOOP && tick == 30;
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "chase-and-shoot.rlog"));

        String stage = SimTelemetry.lastLine(result.telemetry().transmissions(), "Stage");
        String chase = SimTelemetry.lastLine(result.telemetry().transmissions(), "Chase");
        System.out.println("Chase and shoot warnings: " + result.warnings());
        System.out.printf("Chase and shoot: %s; %s; %d shots, %d still held, red tipped %d times, %d in its up cell, "
                + "robot at x %.1f y %.1f%n", stage, chase, sim.shotsFired(), sim.field.getHeldCount(),
                sim.match.tips(Alliance.RED), sim.match.cellBalls(Alliance.RED), sim.x(), sim.y());

        assertTrue(sim.match.tips(Alliance.RED) >= 2, "after a tip it should shoot into the new up cell: " + stage);
    }
}
