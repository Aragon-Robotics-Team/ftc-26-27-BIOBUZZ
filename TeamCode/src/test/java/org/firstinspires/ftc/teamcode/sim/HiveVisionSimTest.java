package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.TeleOp;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.List;

/** HiveVision in the real TeleOp: the Limelight on the turret reads the hive's tags and corrects MatchState.hive. */
class HiveVisionSimTest {
    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void theCameraCorrectsAWrongHive() {
        // Red's hive really has its LEFT (rear) cell up, but TeleOp starts believing RIGHT, as every match does.
        // The tags face the up cell's side, so the robot starts in the rear corner, where it can see them.
        BiobuzzRobotSim sim = new BiobuzzRobotSim(Hive.LEFT, Hive.RIGHT);
        sim.placeAt(RobotConstants.Field.RED_LEFT_CORNER.x(), RobotConstants.Field.RED_LEFT_CORNER.y(),
                RobotConstants.Field.RED_LEFT_CORNER.heading());

        OpModeRunResult result = sim.runner(TeleOp.class)
                .initLoopTicks(40)
                .activeTicks(150) // 3 s; the turret aims at the hive cell TeleOp believes is up
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    // gamepad 2 share in init: "robot is in the LEFT corner" (after init's 25-tick Pinpoint wait)
                    gamepad2.back = phase == OpModePhase.INIT_LOOP && tick == 30; // share is back on the wire
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "hive-vision.rlog"));
        System.out.println("HiveVision warnings: " + result.warnings());

        List<List<String>> sent = result.telemetry().transmissions();
        String says = SimTelemetry.lastLine(sent, "Hive camera says");
        String tags = SimTelemetry.lastLine(sent, "Hive tags");
        System.out.println(tags + " / " + says);
        assertTrue(says.contains("LEFT up"), "the camera should read the LEFT cell up: " + says);
        assertTrue(says.contains("switches 1"), "HiveVision should have switched MatchState.hive once: " + says);
    }
}
