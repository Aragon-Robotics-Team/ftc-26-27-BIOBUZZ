package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.vision.HiveReader;
import org.junit.Test;

import java.util.ArrayDeque;
import java.util.Random;

/**
 * PoseFilter on made-up drives: a robot circling in front of the red hive with its turret on it, odometry whose
 * heading drifts (the shaken IMU), and Limelight frames with noise, late, taken while the hive's arm rests a few
 * degrees short of its stop. The tags come from HiveTags; PoseFilter.measure turns them into what the camera sees.
 */
public class PoseFilterTest {
    private static final PoseFilter.Camera CAMERA = new PoseFilter.Camera(1.02, 0, 4.8, 12.2, Math.toRadians(45));
    private static final double DT = 0.02, FRAME_EVERY = 0.04, LATENCY = 0.035;
    private static final double H_FOV = Math.toRadians(54.5), V_FOV = Math.toRadians(42), MAX_RANGE = 130;

    @Test
    public void armFromHeightUndoesTheTagHeight() {
        for (int id : new int[] {30, 33, 34, 37, 38, 41, 42, 45}) {
            for (double deg = -45; deg <= 45; deg += 7.5) {
                double height = HiveTags.position(id, Math.toRadians(deg))[2];
                assertEquals(id + " at " + deg, deg, Math.toDegrees(PoseFilter.armFromHeight(id, height)), 1e-4);
            }
        }
    }

    @Test
    public void tagsAtRestAreWhereHiveReaderExpectsThem() {
        // Up cell's tags about 49.7 in, down cell's about 35.5 in (HiveReader), on both hives
        assertEquals(49.7, HiveTags.position(34, HiveTags.restAngle(Alliance.RED, Hive.RIGHT))[2], 0.1);
        assertEquals(35.5, HiveTags.position(30, HiveTags.restAngle(Alliance.RED, Hive.RIGHT))[2], 0.1);
        assertEquals(49.7, HiveTags.position(38, HiveTags.restAngle(Alliance.BLUE, Hive.LEFT))[2], 0.1);
        assertEquals(Hive.LEFT, HiveTags.up(Alliance.BLUE, HiveTags.restAngle(Alliance.BLUE, Hive.LEFT)));
        assertEquals(Hive.RIGHT, HiveTags.up(Alliance.RED, HiveTags.restAngle(Alliance.RED, Hive.RIGHT)));
    }

    @Test
    public void cameraFrameMatchesHiveReader() {
        // A robot facing the hive: HiveReader's height from the camera-frame position is the tag's real height
        PoseFilter filter = filter(30, 40, Math.toRadians(70), Math.toRadians(-30));
        double[] s = {30, 40, Math.toRadians(70), Math.toRadians(-30), Math.toRadians(30)};
        double[] seen = filter.measure(s, 0, 35);
        assertTrue(seen != null && seen[2] > 0);
        assertEquals(HiveTags.position(35, Math.toRadians(-30))[2],
                HiveReader.height(seen[1], seen[2], 12.2, Math.toRadians(45)), 1e-9);
    }

    @Test
    public void tagsHoldThePoseAgainstADriftingHeading() {
        Run fused = drive(true, Math.toRadians(-27), Double.NaN, 1);
        Run odometryOnly = drive(false, Math.toRadians(-27), Double.NaN, 1);
        System.out.printf("drift test: odometry %.2f in %.2f deg; fused %.2f in %.2f deg, arm %.2f, used %d rejected %d resets %d%n",
                odometryOnly.positionError, Math.toDegrees(odometryOnly.headingError), fused.positionError,
                Math.toDegrees(fused.headingError), Math.toDegrees(fused.filter.arm(HiveTags.RED)), fused.filter.getUsed(),
                fused.filter.getRejected(), fused.filter.getArmResets());
        // The drift alone throws the pose well off in 30 s ...
        assertTrue("odometry off by " + odometryOnly.positionError, odometryOnly.positionError > 5);
        assertTrue(Math.toDegrees(odometryOnly.headingError) > 10);
        // ... and the tags keep it close, while working out that the arm rests at 27°, not 30°
        assertTrue("fused off by " + fused.positionError, fused.positionError < 1.5);
        assertTrue("heading off by " + Math.toDegrees(fused.headingError), Math.toDegrees(fused.headingError) < 1);
        assertEquals(-27, Math.toDegrees(fused.filter.arm(HiveTags.RED)), 1);
        assertTrue(fused.filter.getUsed() > 300);
        assertEquals("no tip, so the arm never starts over", 0, fused.filter.getArmResets());
    }

    @Test
    public void followsTheArmThroughATip() {
        // The hive tips 15 s in and settles at 28° the other way; the filter starts the arm over and keeps the pose
        Run run = drive(true, Math.toRadians(-27), Math.toRadians(28), 2);
        assertEquals(28, Math.toDegrees(run.filter.arm(HiveTags.RED)), 1);
        assertTrue(run.filter.getArmResets() >= 1);
        assertTrue("off by " + run.positionError, run.positionError < 1.5);
    }

    @Test
    public void throwsOutAMisread() {
        PoseFilter filter = filter(30, 40, Math.toRadians(70), Math.toRadians(-30));
        filter.predict(0.02, 0, 0, 0, 0.02);
        double[] s = {30, 40, Math.toRadians(70), Math.toRadians(-30), Math.toRadians(30)};
        double[] seen = filter.measure(s, 0, 35);
        // 20 in to the side of where the tag is
        assertFalse(filter.addTag(0.02, 0, 35, seen[0] + 20, seen[1], seen[2]));
        assertEquals(30, filter.x(), 1e-9);
        assertTrue(filter.addTag(0.02, 0, 35, seen[0], seen[1], seen[2]));
    }

    // ---- A made-up drive ----

    private static final class Run {
        PoseFilter filter;
        double positionError, headingError;
    }

    private static PoseFilter filter(double x, double y, double heading, double redArm) {
        return new PoseFilter(CAMERA, x, y, heading, redArm, Math.toRadians(30));
    }

    /**
     * 30 s circling in front of the red hive's audience cell (the RIGHT cell up), the turret on the hive. Odometry
     * gets its heading 1°/s off, plus noise; frames every 40 ms, 35 ms late, with 0.5° and 2% noise.
     *
     * @param tipTo the arm's angle after a tip at 15 s, or NaN for no tip
     */
    private static Run drive(boolean useTags, double arm, double tipTo, long seed) {
        Random random = new Random(seed);
        // The filter believes the arm is at its stop (30°): the real one rests short of it
        PoseFilter filter = filter(40, 30, 0, HiveTags.restAngle(Alliance.RED, Hive.RIGHT));
        double x = 40, y = 30, h = 0;                   // truth
        ArrayDeque<double[]> inFlight = new ArrayDeque<>(); // frames waiting out their latency: {ready, taken, id, cx, cy, cz, turret}
        double nextFrame = 0;
        double[] truth = {0, 0, 0, arm, Math.toRadians(30)};
        for (double t = DT; t <= 30 + 1e-9; t += DT) {
            if (!Double.isNaN(tipTo) && t > 15) truth[3] = tipTo;
            // Around a circle 30 in across centred 36 in out from the hive's audience side, facing along it
            double a = t * 0.4;
            double nx = 50 + 15 * Math.cos(a), ny = 30 + 15 * Math.sin(a), nh = a + Math.PI / 2;
            double c = Math.cos(h), s = Math.sin(h);
            double dx = (nx - x) * c + (ny - y) * s, dy = -(nx - x) * s + (ny - y) * c;
            double dh = Math.atan2(Math.sin(nh - h), Math.cos(nh - h));
            x = nx;
            y = ny;
            h = nh;
            // Odometry: the IMU drifts 1°/s and the pods slip a little
            double odx = dx * (1 + 0.01 * random.nextGaussian()), ody = dy + 0.005 * random.nextGaussian();
            double odh = dh + Math.toRadians(1) * DT + Math.toRadians(0.05) * random.nextGaussian();
            filter.predict(t, odx, ody, odh, DT);

            if (useTags && t >= nextFrame) {
                nextFrame = t + FRAME_EVERY;
                truth[0] = x;
                truth[1] = y;
                truth[2] = h;
                double turret = clamp(Math.atan2(72 - (y), HiveTags.pivotX(HiveTags.RED) - x) - h);
                for (int id = 30; id <= 37; id++) {
                    double[] seen = filter.measure(truth, turret, id);
                    if (seen == null || !inView(seen)) continue;
                    double range = Math.sqrt(seen[0] * seen[0] + seen[1] * seen[1] + seen[2] * seen[2]);
                    double across = range * Math.tan(Math.toRadians(0.5));
                    inFlight.addLast(new double[] {t + LATENCY, t, id, seen[0] + across * random.nextGaussian(),
                            seen[1] + across * random.nextGaussian(), seen[2] * (1 + 0.02 * random.nextGaussian()),
                            turret});
                }
            }
            while (!inFlight.isEmpty() && inFlight.peekFirst()[0] <= t) {
                double[] f = inFlight.removeFirst();
                filter.addTag(f[1], f[6], (int) f[2], f[3], f[4], f[5]);
            }
        }
        Run run = new Run();
        run.filter = filter;
        run.positionError = Math.hypot(filter.x() - x, filter.y() - y);
        run.headingError = Math.abs(Math.atan2(Math.sin(filter.heading() - h), Math.cos(filter.heading() - h)));
        return run;
    }

    private static boolean inView(double[] seen) {
        double range = Math.sqrt(seen[0] * seen[0] + seen[1] * seen[1] + seen[2] * seen[2]);
        return range < MAX_RANGE && Math.abs(Math.atan2(seen[0], seen[2])) < H_FOV / 2
                && Math.abs(Math.atan2(seen[1], seen[2])) < V_FOV / 2;
    }

    private static double clamp(double turret) {
        double a = Math.atan2(Math.sin(turret), Math.cos(turret));
        return Math.max(-Math.PI / 2, Math.min(Math.PI / 2, a));
    }
}
