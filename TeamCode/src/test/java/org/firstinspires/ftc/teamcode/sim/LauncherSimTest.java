package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.TeleOp;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;

/**
 * The ball path in the real TeleOp: the driver spins the flywheel up and holds the trigger, which opens the gate and
 * feeds, so the 4 preload pollen fly; then turns the intake on (triangle travels as y) and drives onto a pollen,
 * which it picks up. Logged to launcher.rlog: Shots/... draws each shot's flown arc beside the one that would have
 * gone in.
 */
class LauncherSimTest {
    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void itShootsItsPreloadAndPicksUpAnother() {
        BiobuzzRobotSim sim = new BiobuzzRobotSim();
        double startX = RobotConstants.Field.RED_LEFT_CORNER.x(), startY = RobotConstants.Field.RED_LEFT_CORNER.y();
        sim.placeAt(startX, startY, RobotConstants.Field.RED_LEFT_CORNER.heading());
        // A pollen a foot ahead of the robot's front, where it will drive
        double r = Ball.Type.POLLEN.diameter / 2;
        int pollen = sim.field.addLoosePiece(Ball.Type.POLLEN.name(), FieldFrame.x(startX + 18, startY),
                FieldFrame.y(startX + 18, startY), r);

        sim.runner(TeleOp.class)
                .initLoopTicks(40)
                .activeTicks(330)
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    // gamepad 2 share in init: "robot is in the LEFT corner" (after init's 25-tick Pinpoint wait)
                    gamepad2.back = phase == OpModePhase.INIT_LOOP && tick == 30;
                    boolean active = phase == OpModePhase.ACTIVE;
                    gamepad1.right_bumper = active && tick < 3;                           // flywheel on
                    gamepad1.right_trigger = active && tick >= 100 && tick < 200 ? 1 : 0; // shoot for 2 s
                    gamepad1.y = active && tick >= 220 && tick < 223;                     // triangle: intake on
                    gamepad1.left_stick_y = active && tick >= 230 && tick < 300 ? -0.4f : 0; // forward, slowly
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "launcher.rlog"));

        System.out.printf("Launcher: %d shots, %d held, robot at x %.1f%n", sim.shotsFired(), sim.field.getHeldCount(),
                sim.x());
        assertEquals(4, sim.shotsFired(), "the trigger should have fed all 4 preload pollen");
        // It also picks up any of its shots that landed short on its way
        assertTrue(sim.field.getPiece(pollen).isHeld(), "driving onto the pollen with the intake on should pick it up");
    }
}
