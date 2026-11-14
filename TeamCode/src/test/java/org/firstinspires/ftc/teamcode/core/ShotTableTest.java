package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** ShotTable.Table: SolversLib's InterpLUT made safe for any pasted points, plus the shot range and speed limits. */
public class ShotTableTest {
    private static ShotTable.Table table(double[]... points) {
        return new ShotTable.Table(points);
    }

    @Test
    public void goesThroughEveryPointInAnyOrder() {
        ShotTable.Table t = table(new double[] {60, 1500}, new double[] {40, 1300}, new double[] {80, 1800});
        assertEquals(1300, t.velocityAt(40), 1e-9);
        assertEquals(1500, t.velocityAt(60), 1e-9);
        assertEquals(1800, t.velocityAt(80), 1e-9);
        double between = t.velocityAt(50);
        assertTrue("monotone between rising points: " + between, between > 1300 && between < 1500);
    }

    @Test
    public void holdsTheEndSpeedsPastEitherEnd() {
        ShotTable.Table t = table(new double[] {40, 1300}, new double[] {60, 1500});
        assertEquals(1300, t.velocityAt(30), 1e-9);
        assertEquals(1500, t.velocityAt(70), 1e-9);
    }

    @Test
    public void sameDistancesAverageInsteadOfThrowing() {
        ShotTable.Table t = table(new double[] {40, 1300}, new double[] {40, 1340}, new double[] {60, 1500});
        assertEquals(1320, t.velocityAt(40), 1e-9);
        assertEquals(2, t.size);
    }

    @Test
    public void fewerThanTwoDistancesIsNoTable() {
        assertTrue(Double.isNaN(table().velocityAt(50)));
        assertTrue(Double.isNaN(table(new double[] {50, 1400}).velocityAt(50)));
        assertTrue(Double.isNaN(table(new double[] {50, 1400}, new double[] {50, 1500}).velocityAt(50)));
        assertFalse(table(new double[] {50, 1400}).covers(50));
    }

    @Test
    public void keepsSpeedsWithinTheFlywheel() {
        ShotTable.Table t = table(new double[] {40, 1300}, new double[] {60, 9000});
        assertEquals(RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY, t.velocityAt(60), 1e-9);
    }

    @Test
    public void coversALittlePastEitherEnd() {
        ShotTable.Table t = table(new double[] {40, 1300}, new double[] {80, 1800});
        double extra = RobotConstants.Launcher.SHOT_TABLE_EXTRAPOLATE;
        assertTrue(t.covers(60));
        assertTrue(t.covers(40 - extra + 0.1));
        assertFalse(t.covers(40 - extra - 0.1));
        assertFalse(t.covers(80 + extra + 0.1));
    }

    @Test
    public void theTableInCodeLoads() {
        // Whatever is pasted into POINTS, loading the class must not throw
        ShotTable.size();
        if (ShotTable.isEmpty()) assertTrue(Double.isNaN(ShotTable.velocityAt(40)));
    }
}
