package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.opmodes.TeleOp;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;

/**
 * The hive trays tip from the weight in their up cell, as the Field Setup Guide (12.3) says the real ones do: 8 pollen
 * tip a hive, 3 pollen and 3 nectar tip it, 7 pollen don't. Balls are set down gently in the red hive's up cell, one
 * every half second, as if they'd just been scored, and roll to its back wall.
 */
class HiveTippingSimTest {
    private static final double TICK = 0.02;
    private static final double DROP_EVERY = 0.5; // seconds between balls
    private static final String RED_REAR = SimHiveTrays.cellName(SimHiveTrays.RED, true);

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void sevenPollenDoNotTipIt() {
        GamePieceTracker field = redRearUp();
        drop(field, 7, 0);
        run(field, 4);
        assertEquals(Hive.LEFT, SimHiveTrays.up(false, field.getBodyAngleRad(SimHiveTrays.RED)), "still LEFT up");
        assertEquals(7, field.getScoredCountIn(RED_REAR), "all seven in the cell");
    }

    @Test
    void eightPollenTipItAndSpillOut() {
        GamePieceTracker field = redRearUp();
        drop(field, 8, 0);
        double tippedAt = runUntilRightUp(field, 3);
        System.out.printf("8 pollen: tipped %.2f s after the last one landed%n", DROP_EVERY + tippedAt);
        assertTrue(tippedAt >= 0, "8 pollen should tip it to RIGHT up");
        run(field, 2);
        assertEquals(0, field.getScoredCountIn(RED_REAR), "the cell that came down emptied");
        assertEquals(8, field.getLooseCount(), "and its balls are on the floor");
    }

    @Test
    void threePollenAndThreeNectarTipIt() {
        GamePieceTracker field = redRearUp();
        drop(field, 3, 0);
        for (int i = 3; i < 6; i++) {
            SimHiveTrays.dropIntoUpCell(field, false, Ball.Type.RED_NECTAR, i);
            run(field, DROP_EVERY);
        }
        assertTrue(runUntilRightUp(field, 3) >= 0, "3 pollen + 3 nectar should tip it");
    }

    /**
     * The same in a robot run, to watch in AdvantageScope (hive-tipping.rlog, field "BIOBUZZ Sim (tilting hives)"):
     * TeleOp sits still while pollen land in red's up cell, one every half second, 8 at a time. The match starts with
     * 3 nectar in it, so the 3rd pollen tips it; the rest land in the new up cell, and the second 8 tip it back.
     */
    @Test
    void aRobotRunShowsRedTippingTwice() {
        BiobuzzRobotSim sim = new BiobuzzRobotSim(); // both hives RIGHT up, as every match starts
        sim.placeAt(56, 8, Math.PI / 2);
        double[] midway = new double[1];
        int perBall = (int) Math.round(DROP_EVERY / TICK);
        int secondWave = 400;
        sim.runner(TeleOp.class)
                .initLoopTicks(30)
                .activeTicks(750)
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    // Not gamepad input: this callback runs once a tick, which is when balls land
                    if (phase != OpModePhase.ACTIVE) return;
                    for (int wave : new int[] {50, secondWave}) {
                        int i = (tick - wave) / perBall;
                        if (tick >= wave && (tick - wave) % perBall == 0 && i < 8) {
                            SimHiveTrays.dropIntoUpCell(sim.field, false, Ball.Type.POLLEN, i);
                        }
                    }
                    if (tick == secondWave) midway[0] = sim.field.getBodyAngleRad(SimHiveTrays.RED);
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "hive-tipping.rlog"));
        assertEquals(Hive.LEFT, SimHiveTrays.up(false, midway[0]), "the first pollen tipped it to LEFT up");
        assertEquals(Hive.RIGHT, SimHiveTrays.up(false, sim.field.getBodyAngleRad(SimHiveTrays.RED)),
                "the next 8 tipped it back");
        assertEquals(2, sim.match.tips(org.firstinspires.ftc.teamcode.core.Alliance.RED), "the match counted both tips");
    }

    // ---------------------------------------------------------------------------------------- helpers

    /** Both trays at their stops, red with its rear cell (LEFT) up. */
    private static GamePieceTracker redRearUp() {
        GamePieceTracker field = new GamePieceTracker(4, SimField.world());
        field.setBodyMotion(SimHiveTrays.RED, SimHiveTrays.angle(false, Hive.LEFT), 0);
        field.setBodyMotion(SimHiveTrays.BLUE, SimHiveTrays.angle(true, Hive.RIGHT), 0);
        return field;
    }

    /** Pollen into red's up cell, one every DROP_EVERY, from slot {@code first}. */
    private static void drop(GamePieceTracker field, int count, int first) {
        for (int i = first; i < first + count; i++) {
            SimHiveTrays.dropIntoUpCell(field, false, Ball.Type.POLLEN, i);
            run(field, DROP_EVERY);
        }
    }

    private static void run(GamePieceTracker field, double seconds) {
        for (int t = 0; t < Math.round(seconds / TICK); t++) {
            field.advance(TICK);
        }
    }

    /** Seconds until red's tray reaches the other stop (RIGHT up), or -1 within the limit. */
    private static double runUntilRightUp(GamePieceTracker field, double limit) {
        for (int t = 0; t <= Math.round(limit / TICK); t++) {
            if (SimHiveTrays.up(false, field.getBodyAngleRad(SimHiveTrays.RED)) == Hive.RIGHT) return t * TICK;
            field.advance(TICK);
        }
        return -1;
    }
}
