package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretLimitsTest {
    private static final double MIN = Math.toRadians(-112.5), MAX = Math.toRadians(112.5);

    /** Feeds wanted angles (degrees) through in order, like successive loops; returns each target (degrees). */
    private static double[] replay(double... wantedDeg) {
        double target = 0;
        boolean clipped = false;
        double[] out = new double[wantedDeg.length];
        for (int i = 0; i < wantedDeg.length; i++) {
            double wanted = Math.toRadians(wantedDeg[i]);
            target = TurretLimits.target(wanted, target, clipped, MIN, MAX);
            clipped = target != wanted;
            out[i] = Math.toDegrees(target);
        }
        return out;
    }

    private static void assertTargets(double[] expected, double[] actual) {
        for (int i = 0; i < expected.length; i++) assertEquals("loop " + i, expected[i], actual[i], 1e-9);
    }

    @Test
    public void inRangeItAimsStraightAtTheTarget() {
        assertTargets(new double[] {0, 45, -90, 112}, replay(0, 45, -90, 112));
    }

    @Test
    public void wobblingAcrossStraightBackItStaysOnOneLimit() {
        // The robot turning slowly: the hive goes from the left, around the back (wobbling across 180°), to the right
        assertTargets(new double[] {90, 110, 112.5, 112.5, 112.5, 112.5, 112.5, 112.5, -112},
                replay(90, 110, 115, 150, 179, -179, 179, -150, -112));
    }

    @Test
    public void itParksOnTheLimitTheTargetLeftThrough() {
        assertTargets(new double[] {-100, -112.5, -112.5}, replay(-100, -120, 170));
    }

    @Test
    public void startingInTheBlindZoneItPicksTheNearestLimit() {
        assertTargets(new double[] {112.5}, replay(150));
        assertTargets(new double[] {-112.5}, replay(-150));
    }

    @Test
    public void backInRangeOnTheOtherSideAndOutAgainItParksOnThatSide() {
        assertTargets(new double[] {112.5, -110, -112.5, -112.5}, replay(150, -110, -115, 179));
    }

    @Test
    public void wobblingAtALimitEdgeBarelyMovesIt() {
        assertTargets(new double[] {112.4, 112.5, 112.4, 112.5}, replay(112.4, 112.6, 112.4, 112.6));
    }
}
