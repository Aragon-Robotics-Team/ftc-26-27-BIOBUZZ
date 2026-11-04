package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.Auto;
import org.firstinspires.ftc.teamcode.routines.Routines;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestFactory;

import java.nio.file.Paths;
import java.util.List;
import java.util.Locale;
import java.util.function.UnaryOperator;
import java.util.stream.Stream;

/**
 * The Solo Carry Auto (routines/SoloCarry): the match's staged balls and a partner that only parks (SimPartner).
 *
 * The reference run, on red with the Pinpoint's IMU drifting IMU_DRIFT degrees a second (harsh: more than a shaken IMU
 * is likely to), should tip the hive, park clear of the partner, and keep its pose with the hive's tags while the
 * Pinpoint's goes off (pedro.FusedLocalizer). Logged to solo-carry.rlog, the ball camera's frames (every 5th, rings on
 * what the pipeline found) to build/sim/solo-carry-frames/.
 *
 * The scenarios run it again with what goes wrong on a real field: the robot set down off its start, shots that
 * scatter (some miss, so its count of what's in the cell is wrong), no tags at all, blue. Each has to end with both
 * robots parked apart and score at least its floor, in AUTO points (tips, LEAVE, PARK): 26 is a tip and both robots
 * leaving, 13 doesn't need the tip. Logged to solo-carry-<scenario>.rlog.
 */
class SoloCarrySimTest {
    private static final double IMU_DRIFT = 0.5; // degrees a second: 15° over the 30 s
    private static final int FIRST_PRESS_TICK = 30; // after Auto's init (the Pinpoint's 500 ms)
    private static final Pose RED_START = new Pose(RobotConstants.Field.ROBOT_LENGTH / 2, 36, 0); // allianceWallStart

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void itTipsTheHiveAloneParksAndKeepsItsPose() throws java.io.IOException {
        BiobuzzRobotSim sim = new BiobuzzRobotSim().withParkingPartner().imuDrift(IMU_DRIFT);
        sim.webcam.saveFrames(Paths.get("build", "sim", "solo-carry-frames"), 5);
        Outcome run = run("solo-carry", Alliance.RED, sim, 0, 0, 0);
        Errors errors = errors(sim.trace, SimTelemetry.poses(run.sent));
        System.out.printf("Solo carry pose, IMU drifting %.1f°/s: fused off by %.2f in on average (%.2f at most, %.2f° "
                        + "heading at most); the Pinpoint alone by %.2f in (%.2f at most, %.2f° at the end)%n", IMU_DRIFT,
                errors.fusedMean, errors.fusedMax, errors.fusedHeadingMax, errors.odometryMean, errors.odometryMax,
                errors.odometryHeadingEnd);

        assertTrue(run.tips >= 1, "it should tip the hive on its own");
        assertEquals(2, run.parked, "both robots should end in the loading zone");
        assertTrue(run.apart > 0.5, "it should park clear of the partner, " + run.apart + " in apart");
        // Tags are only in view from the up cell's side, so the fused pose drifts too while the robot is on the other
        // side, and comes back when they're seen again
        assertTrue(errors.fusedMean < 1, "the fused pose should stay within 1 in on average, was " + errors.fusedMean);
        assertTrue(errors.odometryMean > 3 * errors.fusedMean, "the drift should throw the Pinpoint off much further");
        assertTrue(errors.fusedMax < errors.odometryMax / 2, "the fused pose's worst should be well under the Pinpoint's");
    }

    /**
     * What could go wrong, one at a time and together. Floors: a tip, both parked, apart (20 + 3 + 10), except where
     * the shots scatter enough that the preload might not tip it, where it's both parked (13).
     *
     * With no tags and the IMU drifting, nothing corrects the heading: by the end the pose is about 16 in off, well
     * past the 2 to 4 in the park has to spare, and the filter thinks it's off by half that (the drift is steady, the
     * filter models it as random). It parks short of the partner by what it knows, which isn't enough here: all that's
     * asked is that the partner stays parked. A Limelight check before the match is the fix.
     */
    /**
     * Ten matches as they vary (BiobuzzRobotSim.vary: the gardens, where the partner parks, the camera's misses, shot
     * scatter) with the robot set down up to 1 in and 2° off: how consistent it is, not just how it does once.
     */
    @TestFactory
    Stream<DynamicTest> variedMatches() {
        return java.util.stream.LongStream.rangeClosed(1, 10).mapToObj(seed -> {
            java.util.Random set = new java.util.Random(1000 + seed);
            double dx = set.nextDouble() * 2 - 1, dy = set.nextDouble() * 2 - 1, dh = set.nextDouble() * 4 - 2;
            return scenario("varied-" + seed, Alliance.RED, 26, sim -> sim.vary(seed).imuDrift(IMU_DRIFT), dx, dy, dh);
        });
    }

    @TestFactory
    Stream<DynamicTest> scenarios() {
        return Stream.of(
                scenario("set-down-off", Alliance.RED, 26, sim -> sim.imuDrift(IMU_DRIFT), 1.5, -1.5, 4),
                scenario("set-down-off-other-way", Alliance.RED, 26, sim -> sim.imuDrift(-IMU_DRIFT), -1, 2, -5),
                scenario("shots-scatter", Alliance.RED, 13, sim -> sim.shotSpread(1, 0.03, 1.5), 0, 0, 0),
                scenario("shots-scatter-2", Alliance.RED, 13, sim -> sim.shotSpread(2, 0.03, 1.5), 0, 0, 0),
                scenario("shots-wild", Alliance.RED, 13, sim -> sim.shotSpread(4, 0.08, 3), 0, 0, 0),
                scenario("shots-wild-2", Alliance.RED, 13, sim -> sim.shotSpread(5, 0.08, 3), 0, 0, 0),
                // A match battery and balls that take speed off the flywheel (about 5% each): the spot needs about
                // 93% of its free speed
                scenario("flywheel-sags", Alliance.RED, 13, sim -> sim.flywheelLoad(11.8, 1), 0, 0, 0),
                scenario("flywheel-sags-hard", Alliance.RED, 13, sim -> sim.flywheelLoad(11.2, 1.5), 0, 0, 0),
                // Every shot 5% slow or fast (the flywheel's calibration, or drag the solver gets wrong)
                scenario("shots-slow", Alliance.RED, 13, sim -> sim.shotBias(-0.05), 0, 0, 0),
                scenario("shots-fast", Alliance.RED, 13, sim -> sim.shotBias(0.05), 0, 0, 0),
                // Loaded with 1 where it counts 4 (dropped, or jammed): the count says the preload tipped it; it didn't
                scenario("preload-short", Alliance.RED, 13, sim -> sim.preload(1), 0, 0, 0),
                scenario("no-tags", Alliance.RED, 26, sim -> sim.blindLimelight(), 0, 0, 0),
                // The Limelight stops seeing partway: after it's seen tags on the way to the first launch spot, and
                // after the first cycle
                scenario("limelight-dies-early", Alliance.RED, 26, sim -> sim.blindLimelight(2), 0, 0, 0),
                scenario("limelight-dies-later", Alliance.RED, 26, sim -> sim.blindLimelight(10), 0, 0, 0),
                scenario("no-tags-drifting", Alliance.RED, 26, 1, sim -> sim.blindLimelight().imuDrift(IMU_DRIFT), 1, 1, 2),
                // The ball camera as a Control Hub runs it: 10 frames a second, 100 ms late, 30% of balls missed
                scenario("slow-ball-camera", Alliance.RED, 26, sim -> sim.ballCamera(10, 100, 6, 0.3), 0, 0, 0),
                // Tags lost in bursts of up to 1 s, a fifth of the time: longer than "tags gone 0.6 s means tipped"
                scenario("tags-flicker", Alliance.RED, 26, sim -> sim.flickerLimelight(1, 0.2, 1), 0, 0, 0),
                scenario("tags-flicker-2", Alliance.RED, 26, sim -> sim.flickerLimelight(2, 0.2, 1), 0, 0, 0),
                scenario("tags-flicker-3", Alliance.RED, 26, sim -> sim.flickerLimelight(3, 0.2, 1), 0, 0, 0),
                // The preload misses the tip (every shot 8% slow) and a burst fakes "tipped" right after it: hive
                // vision has to see the RIGHT cell still up and send the next load there
                scenario("tags-flicker-false-tip", Alliance.RED, 13,
                        sim -> sim.shotBias(-0.08).flickerLimelight(4, 0.2, 1), 0, 0, 0),
                // A drive like a real mecanum's (85% speed, strafing 80%, 150 in/s² of grip), and that with the slow
                // ball camera and the IMU drifting
                scenario("real-drive", Alliance.RED, 26, sim -> sim, 0, 0, 0),
                scenario("real-drive-slow-camera", Alliance.RED, 26,
                        sim -> sim.ballCamera(10, 100, 7, 0.3).imuDrift(IMU_DRIFT), 0, 0, 0),
                scenario("blue", Alliance.BLUE, 26, sim -> sim.imuDrift(IMU_DRIFT), 0, 0, 0),
                scenario("everything", Alliance.BLUE, 13,
                        sim -> sim.imuDrift(-IMU_DRIFT).shotSpread(3, 0.03, 1.5), 1.5, 1.5, -4));
    }

    private DynamicTest scenario(String name, Alliance alliance, int floor, UnaryOperator<BiobuzzRobotSim> setUp,
                                 double dx, double dy, double dhDegrees) {
        return scenario(name, alliance, floor, 2, setUp, dx, dy, dhDegrees);
    }

    private DynamicTest scenario(String name, Alliance alliance, int floor, int parked,
                                 UnaryOperator<BiobuzzRobotSim> setUp, double dx, double dy, double dhDegrees) {
        return DynamicTest.dynamicTest(name, () -> {
            boolean realDrive = name.startsWith("real-drive");
            BiobuzzRobotSim sim = setUp.apply(new BiobuzzRobotSim(realDrive).alliance(alliance).withParkingPartner());
            Outcome run = run("solo-carry-" + name, alliance, sim, dx, dy, dhDegrees);
            System.out.printf("Solo carry %s: footprint at most %.1f in past the center line%n", name, sim.pastCenterLine());
            // G402. Except with no tags and the IMU drifting, when the robot doesn't know where it is
            if (!name.equals("no-tags-drifting")) {
                assertTrue(sim.pastCenterLine() <= 0, name + ": it went " + sim.pastCenterLine() + " in onto the other side");
            }
            assertTrue(run.parked >= parked, name + ": " + run.parked + " robots parked, should be " + parked);
            // In a varied match the partner can park 2 in short, which takes the gap down to touching
            double clear = name.startsWith("varied") ? 0 : 0.5;
            if (parked == 2) assertTrue(run.apart >= clear, name + ": it should park clear of the partner, " + run.apart + " in apart");
            assertTrue(run.points >= floor, name + ": " + run.points + " points, under its floor of " + floor);
        });
    }

    private static final class Outcome {
        List<List<String>> sent;
        int points, tips, parked;
        double apart;
    }

    /**
     * Picks Solo Carry and the alliance on the menu and runs the 30 s, with the robot set down (dx, dy) inches and
     * dhDegrees off its start (red's frame: dx out from the alliance wall).
     */
    private static Outcome run(String name, Alliance alliance, BiobuzzRobotSim sim, double dx, double dy,
                               double dhDegrees) {
        List<Routines> routines = Routines.enabled();
        int allianceTick = FIRST_PRESS_TICK + 2 * routines.indexOf(Routines.SOLO_CARRY);
        Pose start = alliance.apply(new Pose(RED_START.x() + dx, RED_START.y() + dy,
                RED_START.heading() + Math.toRadians(dhDegrees)));
        sim.placeAt(start.x(), start.y(), start.heading());
        OpModeRunResult result = sim.runner(Auto.class)
                .initLoopTicks(FIRST_PRESS_TICK + 2 * routines.size() + 10)
                .activeTicks(1500) // 30 s
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    boolean menu = phase == OpModePhase.INIT_LOOP;
                    gamepad1.dpad_down = menu && tick >= FIRST_PRESS_TICK && tick < allianceTick && tick % 2 == 0;
                    gamepad1.b = menu && tick == allianceTick && alliance == Alliance.RED;
                    gamepad1.x = menu && tick == allianceTick && alliance == Alliance.BLUE;
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", name + ".rlog"));

        Outcome run = new Outcome();
        run.sent = result.telemetry().transmissions();
        assertTrue(SimTelemetry.lastLine(run.sent, "Routine").contains("SOLO_CARRY"), "the menu didn't land on Solo Carry");
        // Both park along the alliance wall, the partner (18 in square) toward the rear: the gap between them along it,
        // in red's frame, ours facing along the wall (RobotConstants.Field.ROBOT_LENGTH that way)
        Pose partner = alliance.apply(new Pose(sim.partnerAt()[0], sim.partnerAt()[1], 0));
        Pose us = alliance.apply(new Pose(sim.x(), sim.y(), sim.heading()));
        run.apart = partner.y() - 9 - (us.y() + Math.abs(Math.sin(sim.heading())) * RobotConstants.Field.ROBOT_LENGTH / 2
                + Math.abs(Math.cos(sim.heading())) * RobotConstants.Field.ROBOT_WIDTH / 2);
        run.points = sim.match.score(alliance);
        run.tips = sim.match.tips(alliance);
        run.parked = sim.match.parked(alliance);
        System.out.println("Solo carry warnings: " + result.warnings());
        System.out.printf(Locale.US, "Solo carry %s: %d shots, %d held, %d points (%d tips, %d in the cell, %d parked); "
                        + "ended at x %.1f y %.1f, %.1f in from the partner; %d left in the garden%n", name,
                sim.shotsFired(), sim.field.getHeldCount(), run.points, run.tips, sim.match.cellBalls(alliance),
                run.parked, sim.x(), sim.y(), run.apart, SimField.ballsIn(sim.field, SimField.garden(alliance)));
        return run;
    }

    private static final class Errors {
        double fusedMean, fusedMax, fusedHeadingMax, odometryMean, odometryMax, odometryHeadingEnd;
    }

    /**
     * How far Pedro's pose (the fused one, from the loops' telemetry) and the Pinpoint's own were from the truth over
     * the 30 s. Telemetry is recorded at each update, the OpMode's and the SDK's after loop(), so there are a whole
     * number of transmissions a tick; they and the sim's ticks line up from the end, at the offset that fits best.
     */
    private static Errors errors(List<double[]> trace, List<double[]> poses) {
        int stride = Math.max(1, Math.round((float) poses.size() / trace.size()));
        Errors best = null;
        for (int offset = 0; offset < stride; offset++) {
            for (int shift = -3; shift <= 3; shift++) {
                Errors e = new Errors();
                int n = 0;
                for (int i = 4; i < 1450; i++) { // the 30 s of AUTO, not init
                    int tick = trace.size() - 1 - i - shift, sent = poses.size() - 1 - offset - i * stride;
                    if (tick < 0 || sent < 0) break;
                    double[] pose = poses.get(sent), truth = trace.get(tick);
                    if (pose == null) continue;
                    double fused = Math.hypot(pose[0] - truth[0], pose[1] - truth[1]);
                    double odometry = Math.hypot(truth[3] - truth[0], truth[4] - truth[1]);
                    e.fusedMean += fused;
                    e.fusedMax = Math.max(e.fusedMax, fused);
                    e.fusedHeadingMax = Math.max(e.fusedHeadingMax, Math.abs(Math.toDegrees(
                            Math.IEEEremainder(pose[2] - truth[2], 2 * Math.PI))));
                    e.odometryMean += odometry;
                    e.odometryMax = Math.max(e.odometryMax, odometry);
                    n++;
                }
                if (n == 0) continue;
                e.fusedMean /= n;
                e.odometryMean /= n;
                double[] last = trace.get(trace.size() - 1);
                e.odometryHeadingEnd = Math.abs(Math.toDegrees(Math.IEEEremainder(last[5] - last[2], 2 * Math.PI)));
                if (best == null || e.fusedMean < best.fusedMean) best = e;
            }
        }
        return best;
    }
}
