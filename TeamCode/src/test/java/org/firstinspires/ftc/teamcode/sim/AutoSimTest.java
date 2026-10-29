package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.Auto;
import org.firstinspires.ftc.teamcode.routines.Routines;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;

import java.nio.file.Paths;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;

/**
 * The real Auto on the simulated robot, with a scripted alliance partner (SimPartner) and the match's staged balls:
 * every routine in the menu, each to its own build/sim/auto-&lt;routine&gt;-&lt;alliance&gt;.rlog. On red unless
 * {@code -PsimAlliance=blue}; {@code -PsimRoutine=one_cycle_garden_park} runs just that one.
 * tools/advantagescope/replay_auto.py runs this and opens the result in AdvantageScope.
 */
class AutoSimTest {
    private static final Alliance ALLIANCE =
            "blue".equalsIgnoreCase(System.getProperty("simAlliance")) ? Alliance.BLUE : Alliance.RED;
    private static final String ONLY_ROUTINE = System.getProperty("simRoutine", "all");

    // Where every menu routine starts on red (routines/FieldPoses: start). Checked against Auto's "Place robot at".
    private static final Pose RED_START = new Pose(56, 8, Math.toRadians(90));

    // Auto's init() takes 25 ticks (the Pinpoint localizer's 500 ms sleep), so the menu is driven after that
    private static final int FIRST_PRESS_TICK = 30;

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @TestFactory
    Stream<DynamicTest> everyRoutineDrivesAndThePinpointKeepsUp() {
        return Routines.enabled().stream()
                // Solo Carry starts elsewhere and has its own test (SoloCarrySimTest)
                .filter(routine -> routine != Routines.SOLO_CARRY)
                .filter(routine -> ONLY_ROUTINE.equalsIgnoreCase("all") || ONLY_ROUTINE.equalsIgnoreCase(routine.name()))
                .map(routine -> DynamicTest.dynamicTest(routine.toString(), () -> autoDrivesAndThePinpointKeepsUp(routine)));
    }

    private void autoDrivesAndThePinpointKeepsUp(Routines routine) {
        List<Routines> routines = Routines.enabled();
        int menuIndex = routines.indexOf(routine);
        // Pressed one every other tick, so each counts as a new press: dpad down to the routine, then the alliance
        int allianceTick = FIRST_PRESS_TICK + 2 * menuIndex;
        // The same init length for every routine, so their Autos start at the same time in the logs
        int initTicks = FIRST_PRESS_TICK + 2 * routines.size() + 10;

        Pose start = ALLIANCE.apply(RED_START);
        BiobuzzRobotSim sim = new BiobuzzRobotSim().alliance(ALLIANCE).withPartner();
        sim.placeAt(start.x(), start.y(), start.heading());

        String name = (routine + "-" + ALLIANCE).toLowerCase(Locale.US);
        OpModeRunResult result = sim.runner(Auto.class)
                .initLoopTicks(initTicks)
                .activeTicks(1500) // 30 s
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    boolean menu = phase == OpModePhase.INIT_LOOP;
                    gamepad1.dpad_down = menu && tick >= FIRST_PRESS_TICK && tick < allianceTick && tick % 2 == 0;
                    gamepad1.x = menu && tick == allianceTick && ALLIANCE == Alliance.BLUE;
                    gamepad1.b = menu && tick == allianceTick && ALLIANCE == Alliance.RED;
                }))
                // open in AdvantageScope
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "auto-" + name + ".rlog"));
        System.out.println("Auto warnings: " + result.warnings());
        System.out.printf("Auto %s ended at x %.1f y %.1f h %.0f°%n", name, sim.x(), sim.y(), Math.toDegrees(sim.heading()));
        System.out.printf("Auto %s: %d shots, %d held, %s %d points (%d tips, %d in the cell)%n", name, sim.shotsFired(),
                sim.field.getHeldCount(), ALLIANCE, sim.match.score(ALLIANCE), sim.match.tips(ALLIANCE),
                sim.match.cellBalls(ALLIANCE));
        // Our preload and the partner's (SimPartner), 4 pollen each, land in the up cell on top of the 3 staged nectar
        assertTrue(sim.match.tips(ALLIANCE) >= 1, "the two preloads should tip our hive");

        List<List<String>> sent = result.telemetry().transmissions();
        String routineLine = SimTelemetry.lastLine(sent, "Routine");
        assertTrue(routineLine.contains(routine.toString()), "the menu didn't land on " + routine + ": " + routineLine);
        double[] place = SimTelemetry.placeRobotAt(sent);
        assertEquals(start.x(), place[0], 0.1, "Auto's start isn't FieldPoses.start any more: update RED_START");
        assertEquals(start.y(), place[1], 0.1, "Auto's start isn't FieldPoses.start any more: update RED_START");
        assertEquals(0, Math.IEEEremainder(Math.toDegrees(start.heading()) - place[2], 360), 0.5,
                "Auto's start isn't FieldPoses.start any more: update RED_START");

        double moved = Math.hypot(sim.x() - start.x(), sim.y() - start.y());
        assertTrue(moved > 12, "Auto should drive off its start, moved " + moved + " in");

        double[] pedro = SimTelemetry.lastPose(sent);
        assertEquals(sim.x(), pedro[0], 2.0, "Pedro's x should match where the robot really is");
        assertEquals(sim.y(), pedro[1], 2.0, "Pedro's y should match where the robot really is");
    }
}
