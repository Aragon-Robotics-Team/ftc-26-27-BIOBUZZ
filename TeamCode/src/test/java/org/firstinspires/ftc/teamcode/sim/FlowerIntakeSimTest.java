package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.opmodes.TeleOp;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;

/**
 * Taking pollen out of a flower's Retrieval Opening (G418.B) in the real TeleOp: the driver drives square up to the
 * rear wall's flower and holds the robot against it with the intake on, and the 4 staged pollen come out one by one
 * from the bottom, each dropping into the lower ring as the one under it goes. With the intake off the stack stays put.
 * Logged to flower-intake.rlog.
 */
class FlowerIntakeSimTest {
    private static final int REAR_FLOWER = 0; // RobotConstants.Field.FLOWERS[0]: Pedro (48, 140.5)

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void itTakesAFlowersPollenFromTheBottom() {
        BiobuzzRobotSim sim = drive(true, "flower-intake.rlog");
        System.out.printf("Flower intake: %d held, %d left in the flower, robot at y %.1f%n",
                sim.field.getHeldCount(), inFlower(sim), sim.y());
        assertEquals(4, sim.field.getHeldCount(), "the intake should have taken all 4 of the flower's pollen");
        assertEquals(0, inFlower(sim), "the flower should be empty");
    }

    @Test
    void withTheIntakeOffTheStackStays() {
        BiobuzzRobotSim sim = drive(false, "flower-no-intake.rlog");
        assertEquals(0, sim.field.getHeldCount(), "nothing should come out with the intake off");
        assertEquals(4, inFlower(sim), "all 4 pollen should still be in the flower");
    }

    /**
     * Facing the rear wall's flower 10 in short of it, empty, and driven forward slowly into it for 3 s, then held
     * there. The Limelight is off and the pose reset to the LEFT corner, so stick forward is the robot's forward.
     */
    private static BiobuzzRobotSim drive(boolean intake, String log) {
        BiobuzzRobotSim sim = new BiobuzzRobotSim().preload(0).blindLimelight();
        sim.placeAt(48, 120, Math.PI / 2);
        sim.runner(TeleOp.class)
                .initLoopTicks(40)
                .activeTicks(250)
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    gamepad2.back = phase == OpModePhase.INIT_LOOP && tick == 30; // "robot is in the LEFT corner"
                    boolean active = phase == OpModePhase.ACTIVE;
                    gamepad1.y = intake && active && tick >= 2 && tick < 5;          // triangle: intake on
                    gamepad1.left_stick_y = active && tick >= 10 && tick < 160 ? -0.3f : 0;
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", log));
        return sim;
    }

    /** Loose balls inside the flower's tube. */
    private static int inFlower(BiobuzzRobotSim sim) {
        double[] c = SimField.flowerCenter(REAR_FLOWER);
        int n = 0;
        for (GamePiece p : sim.field.getPieces()) {
            if (p.isVisibleOnField() && !p.isHeld()
                    && Math.hypot(p.getFieldXInches() - c[0], p.getFieldYInches() - c[1]) < SimField.FLOWER_INNER_RADIUS) n++;
        }
        return n;
    }
}
