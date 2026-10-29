package org.firstinspires.ftc.teamcode.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.Auto;
import org.firstinspires.ftc.teamcode.routines.Routines;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.nio.file.Paths;
import java.util.List;

/**
 * The Solo Carry Auto (routines/SoloCarry) on red, the match's staged balls, a partner that only parks (SimPartner),
 * and the Pinpoint's IMU drifting IMU_DRIFT degrees a second (harsh: more than a shaken IMU is likely to). It should
 * tip the hive, park clear of the partner, and keep its pose with the hive's tags while the Pinpoint's goes off
 * (pedro.FusedLocalizer). Logged to solo-carry.rlog, the ball camera's frames (every 5th, rings on what the pipeline
 * found) to build/sim/solo-carry-frames/.
 */
class SoloCarrySimTest {
    private static final double IMU_DRIFT = 0.5; // degrees a second: 15° over the 30 s
    private static final int FIRST_PRESS_TICK = 30; // after Auto's init (the Pinpoint's 500 ms)

    @BeforeAll
    static void loadOpenCv() {
        nu.pattern.OpenCV.loadLocally(); // BallVision makes OpenCV Mats when the Robot is built
    }

    @Test
    void itTipsTheHiveAloneParksAndKeepsItsPose() throws java.io.IOException {
        List<Routines> routines = Routines.enabled();
        int menuIndex = routines.indexOf(Routines.SOLO_CARRY);
        int allianceTick = FIRST_PRESS_TICK + 2 * menuIndex;
        BiobuzzRobotSim sim = new BiobuzzRobotSim().withParkingPartner().imuDrift(IMU_DRIFT);
        sim.placeAt(RobotConstants.Field.ROBOT_LENGTH / 2, 36, 0); // FieldPoses.allianceWallStart

        sim.webcam.saveFrames(Paths.get("build", "sim", "solo-carry-frames"), 5);
        OpModeRunResult result = sim.runner(Auto.class)
                .initLoopTicks(FIRST_PRESS_TICK + 2 * routines.size() + 10)
                .activeTicks(1500) // 30 s
                .gamepads(sim.drivers((phase, tick, gamepad1, gamepad2) -> {
                    boolean menu = phase == OpModePhase.INIT_LOOP;
                    gamepad1.dpad_down = menu && tick >= FIRST_PRESS_TICK && tick < allianceTick && tick % 2 == 0;
                    gamepad1.b = menu && tick == allianceTick;
                }))
                .runLogged(BiobuzzRobotSim.ROBOT_CONFIG, Paths.get("build", "sim", "solo-carry.rlog"));

        List<List<String>> sent = result.telemetry().transmissions();
        assertTrue(SimTelemetry.lastLine(sent, "Routine").contains("SOLO_CARRY"), "the menu didn't land on Solo Carry");
        // Both park along the alliance wall, the partner (18 in square) toward the rear: the gap between them along it,
        // ours facing along the wall (RobotConstants.Field.ROBOT_LENGTH that way)
        double[] partner = sim.partnerAt();
        double apart = partner[1] - 9 - (sim.y() + Math.abs(Math.sin(sim.heading())) * RobotConstants.Field.ROBOT_LENGTH / 2
                + Math.abs(Math.cos(sim.heading())) * RobotConstants.Field.ROBOT_WIDTH / 2);
        int garden = SimField.ballsIn(sim.field, SimField.garden(Alliance.RED));
        Errors errors = errors(sim.trace, SimTelemetry.poses(sent));
        System.out.println("Solo carry warnings: " + result.warnings());
        System.out.printf("Solo carry: %d shots, %d held, %d points (%d tips, %d in the cell, %d parked); ended at x %.1f"
                        + " y %.1f, %.1f in from the partner; %d left in the garden%n", sim.shotsFired(), sim.field.getHeldCount(),
                sim.match.score(Alliance.RED), sim.match.tips(Alliance.RED), sim.match.cellBalls(Alliance.RED),
                sim.match.parked(Alliance.RED), sim.x(), sim.y(), apart, garden);
        System.out.printf("Solo carry pose, IMU drifting %.1f°/s: fused off by %.2f in on average (%.2f at most, %.2f° "
                        + "heading at most); the Pinpoint alone by %.2f in (%.2f at most, %.2f° at the end)%n", IMU_DRIFT,
                errors.fusedMean, errors.fusedMax, errors.fusedHeadingMax, errors.odometryMean, errors.odometryMax,
                errors.odometryHeadingEnd);

        assertTrue(sim.match.tips(Alliance.RED) >= 1, "it should tip the hive on its own");
        assertEquals(2, sim.match.parked(Alliance.RED), "both robots should end in the loading zone");
        assertTrue(apart > 0.5, "it should park clear of the partner, " + apart + " in apart");
        // Tags are only in view from the up cell's side, so the fused pose drifts too while the robot is on the other
        // side, and comes back when they're seen again
        assertTrue(errors.fusedMean < 1, "the fused pose should stay within 1 in on average, was " + errors.fusedMean);
        assertTrue(errors.odometryMean > 3 * errors.fusedMean, "the drift should throw the Pinpoint off much further");
        assertTrue(errors.fusedMax < errors.odometryMax / 2, "the fused pose's worst should be well under the Pinpoint's");
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
        System.out.printf("Solo carry: %d transmissions, %d ticks, %d a tick%n", poses.size(), trace.size(), stride);
        return best;
    }
}
