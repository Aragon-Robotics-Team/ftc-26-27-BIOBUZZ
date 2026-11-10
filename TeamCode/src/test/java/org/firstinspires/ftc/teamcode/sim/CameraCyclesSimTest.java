package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Robot;
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
import java.util.function.UnaryOperator;
import java.util.stream.LongStream;
import java.util.stream.Stream;

/**
 * The Camera Cycles Auto (routines/CameraCycles) with the match's staged balls, on the drive like a real mecanum's
 * (BiobuzzRobotSim(true)). Two partners: one that shoots its preloads after our first tip and parks at the audience end
 * of the loading zone (doc/auto-pathing.md; the routine's default), and one that only parks (set with triangle during
 * init). Each run has to stay on our half (G402), park, and score at least its floor in AUTO points (tips, LEAVE,
 * PARK). Logged to build/sim/camera-cycles-&lt;scenario&gt;.rlog.
 */
class CameraCyclesSimTest {
    private static final int FIRST_PRESS_TICK = 30; // after Auto's init (the Pinpoint's 500 ms)
    private static final double IMU_DRIFT = 0.1;    // degrees a second

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    /** Floors: 2 tips, both robots leaving and parked (40 + 6 + 10) with a partner that shoots; 1 tip alone (20 + 16). */
    @TestFactory
    Stream<DynamicTest> scenarios() {
        return Stream.of(
                scenario("partner-shoots", Alliance.RED, true, 56, sim -> sim),
                scenario("partner-shoots-blue", Alliance.BLUE, true, 56, sim -> sim),
                scenario("partner-parks", Alliance.RED, false, 36, sim -> sim),
                scenario("slow-ball-camera", Alliance.RED, true, 56, sim -> sim.ballCamera(10, 100, 6, 0.3)),
                scenario("no-tags", Alliance.RED, true, 56, sim -> sim.blindLimelight()),
                // The flowers need the pose within about 2 in: at 0.2°/s and more it misses the rear one (1 tip)
                scenario("imu-drift", Alliance.RED, true, 56, sim -> sim.imuDrift(IMU_DRIFT)),
                scenario("shots-scatter", Alliance.RED, true, 36, sim -> sim.shotSpread(1, 0.03, 1.5)));
    }

    /** Ten matches as they vary (BiobuzzRobotSim.vary): gardens laid by hand, camera misses, shot scatter. */
    @TestFactory
    Stream<DynamicTest> variedMatches() {
        return LongStream.rangeClosed(1, 10).mapToObj(seed ->
                scenario("varied-" + seed, Alliance.RED, true, 36, sim -> sim.vary(seed)));
    }

    private DynamicTest scenario(String name, Alliance alliance, boolean partnerShoots, int floor,
                                 UnaryOperator<BiobuzzRobotSim> setUp) {
        return DynamicTest.dynamicTest(name, () -> {
            BiobuzzRobotSim sim = new BiobuzzRobotSim(true).alliance(alliance);
            sim = setUp.apply(partnerShoots ? sim.withPartnerAfterTip() : sim.withParkingPartner());
            int points = run("camera-cycles-" + name, alliance, partnerShoots, sim);
            assertTrue(sim.pastCenterLine() <= 0, name + ": it went " + sim.pastCenterLine() + " in onto the other side");
            assertTrue(sim.match.parked(alliance) == 2, name + ": " + sim.match.parked(alliance) + " robots parked");
            assertTrue(points >= floor, name + ": " + points + " points, under its floor of " + floor);
        });
    }

    /** Picks Camera Cycles, the alliance and the partner on the menu and runs the 30 s; the AUTO points. */
    private static int run(String name, Alliance alliance, boolean partnerShoots, BiobuzzRobotSim sim) {
        List<Routines> routines = Routines.enabled();
        int allianceTick = FIRST_PRESS_TICK + 2 * routines.indexOf(Routines.CAMERA_CYCLES);
        int partnerTick = allianceTick + 2;
        Pose start = Routines.CAMERA_CYCLES.create(new Robot(new BiobuzzRobotSim().alliance(alliance).hardwareMap),
                alliance).startPose();
        sim.placeAt(start.x(), start.y(), start.heading());
        OpModeRunResult result = sim.runner(Auto.class)
                .initLoopTicks(FIRST_PRESS_TICK + 2 * routines.size() + 10)
                .activeTicks(1500) // 30 s
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    boolean menu = phase == OpModePhase.INIT_LOOP;
                    gamepad1.dpad_down = menu && tick >= FIRST_PRESS_TICK && tick < allianceTick && tick % 2 == 0;
                    gamepad1.b = menu && tick == allianceTick && alliance == Alliance.RED;
                    gamepad1.x = menu && tick == allianceTick && alliance == Alliance.BLUE;
                    gamepad1.y = menu && tick == partnerTick && !partnerShoots; // triangle: the partner doesn't shoot
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", name + ".rlog"));

        List<List<String>> sent = result.telemetry().transmissions();
        assertTrue(SimTelemetry.lastLine(sent, "Routine").contains("CAMERA_CYCLES"), "the menu didn't land on Camera Cycles");
        String partner = SimTelemetry.lastLine(sent, "Partner");
        assertTrue(partner.contains(partnerShoots ? "shoots" : "doesn't"), "the partner pick didn't take: " + partner);
        int points = sim.match.score(alliance);
        // What it did, stage by stage, with the pose it thought it had then
        String last = "";
        for (int i = 0; i < sent.size(); i++) {
            String doing = null, pose = "";
            for (String line : sent.get(i)) {
                if (line.startsWith("Doing")) doing = line.replaceAll(", held.*", ")");
                if (line.startsWith("Pose")) pose = line;
            }
            String what = doing == null ? null : doing.replaceAll("\\(.*", "");
            if (what != null && !what.equals(last)) {
                java.util.regex.Matcher m = java.util.regex.Pattern.compile("\\(([0-9.]+) s").matcher(doing);
                String truth = "";
                if (m.find()) {
                    int tick = sim.trace.size() - 1500 + (int) Math.round(Double.parseDouble(m.group(1)) * 50);
                    if (tick >= 0 && tick < sim.trace.size()) {
                        double[] t = sim.trace.get(tick);
                        truth = String.format(Locale.US, "truth (%.1f, %.1f, %.1f°)", t[0], t[1], Math.toDegrees(t[2]));
                    }
                }
                System.out.printf(Locale.US, "  %s %s   %s   %s%n", name, doing, pose, truth);
                last = what;
            }
        }
        System.out.println("Camera cycles warnings: " + result.warnings());
        System.out.printf(Locale.US, "Camera cycles %s: %d points (%d tips, %d in the up cell, %d parked), %d shots, "
                        + "%d held at the end, at x %.1f y %.1f; %d left in the garden; last: %s%n", name, points,
                sim.match.tips(alliance), sim.match.cellBalls(alliance), sim.match.parked(alliance), sim.shotsFired(),
                sim.field.getHeldCount(), sim.x(), sim.y(), SimField.ballsIn(sim.field, SimField.garden(alliance)),
                lastDoing(sent));
        return points;
    }

    /** Auto's last "Doing" line: the routine's status. */
    private static String lastDoing(List<List<String>> sent) {
        for (int i = sent.size() - 1; i >= 0; i--) {
            for (String line : sent.get(i)) if (line.startsWith("Doing")) return line;
        }
        return "none";
    }
}
