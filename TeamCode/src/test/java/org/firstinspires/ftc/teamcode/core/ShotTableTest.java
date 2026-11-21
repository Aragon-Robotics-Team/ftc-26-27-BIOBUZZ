package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ShotTableTest {
    private static List<ShotTable.Entry> table(double[]... rows) {
        List<ShotTable.Entry> table = new ArrayList<>();
        for (double[] row : rows) ShotTable.addTo(table, new ShotTable.Entry(row[0], row[1]));
        return table;
    }

    @Test
    public void interpolatesBetweenEntries() {
        List<ShotTable.Entry> t = table(new double[] {60, 1500}, new double[] {40, 1300}, new double[] {80, 1800});
        assertEquals(1300, ShotTable.velocityAt(t, 40), 1e-9);
        assertEquals(1400, ShotTable.velocityAt(t, 50), 1e-9);
        assertEquals(1650, ShotTable.velocityAt(t, 70), 1e-9);
        assertEquals(1800, ShotTable.velocityAt(t, 80), 1e-9);
    }

    @Test
    public void extrapolatesFromTheEndSegments() {
        List<ShotTable.Entry> t = table(new double[] {40, 1300}, new double[] {60, 1500});
        assertEquals(1200, ShotTable.velocityAt(t, 30), 1e-9);
        assertEquals(1600, ShotTable.velocityAt(t, 70), 1e-9);
    }

    @Test
    public void capsAtMaxVelocity() {
        List<ShotTable.Entry> t = table(new double[] {40, 1300}, new double[] {60, 2000});
        assertEquals(RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY, ShotTable.velocityAt(t, 200), 1e-9);
    }

    @Test
    public void emptyAndSingle() {
        assertTrue(Double.isNaN(ShotTable.velocityAt(new ArrayList<>(), 50)));
        assertFalse(ShotTable.covers(new ArrayList<>(), 50));
        assertEquals(1400, ShotTable.velocityAt(table(new double[] {50, 1400}), 90), 1e-9);
    }

    @Test
    public void coversOnlyNearTheMeasuredRange() {
        List<ShotTable.Entry> t = table(new double[] {40, 1300}, new double[] {80, 1800});
        double extra = RobotConstants.Launcher.SHOT_TABLE_EXTRAPOLATE;
        assertTrue(ShotTable.covers(t, 60));
        assertTrue(ShotTable.covers(t, 40 - extra + 0.1));
        assertFalse(ShotTable.covers(t, 40 - extra - 0.1));
        assertFalse(ShotTable.covers(t, 80 + extra + 0.1));
    }

    @Test
    public void keepsEverySaveInDistanceOrder() {
        List<ShotTable.Entry> t = table(new double[] {40, 1300}, new double[] {60, 1500});
        ShotTable.addTo(t, new ShotTable.Entry(41, 1350));
        ShotTable.addTo(t, new ShotTable.Entry(40, 1320));
        ShotTable.addTo(t, new ShotTable.Entry(50, 1420));
        assertEquals(5, t.size());
        assertEquals(Arrays.asList(40.0, 40.0, 41.0, 50.0, 60.0), Arrays.asList(t.get(0).distance,
                t.get(1).distance, t.get(2).distance, t.get(3).distance, t.get(4).distance));
    }

    @Test
    public void averagesSavesAtAboutTheSameDistance() {
        double savedMerge = RobotConstants.Launcher.SHOT_TABLE_MERGE;
        try {
            RobotConstants.Launcher.SHOT_TABLE_MERGE = 3;
            // Three noisy saves around 40 in, two around 60 in
            List<ShotTable.Entry> t = table(new double[] {39, 1280}, new double[] {40, 1300},
                    new double[] {41, 1320}, new double[] {59, 1480}, new double[] {61, 1520});
            List<double[]> curve = ShotTable.curve(t);
            assertEquals(2, curve.size());
            assertEquals(40, curve.get(0)[0], 1e-9);
            assertEquals(1300, curve.get(0)[1], 1e-9);
            assertEquals(3, curve.get(0)[2], 1e-9);
            assertEquals(60, curve.get(1)[0], 1e-9);
            assertEquals(1500, curve.get(1)[1], 1e-9);
            // No zig-zag between the 40 in saves: a straight line between the averages
            assertEquals(1310, ShotTable.velocityAt(t, 41), 1e-9);
            assertEquals(1400, ShotTable.velocityAt(t, 50), 1e-9);

            RobotConstants.Launcher.SHOT_TABLE_MERGE = 0; // every save its own point
            assertEquals(5, ShotTable.curve(t).size());
        } finally {
            RobotConstants.Launcher.SHOT_TABLE_MERGE = savedMerge;
        }
    }

    @Test
    public void csvRoundTrip() {
        ShotTable.Entry e = new ShotTable.Entry(52.25, 1480, 30.5, 70.25, Math.toRadians(15), "RED", "LEFT");
        ShotTable.Entry back = ShotTable.parse(ShotTable.format(e));
        assertNotNull(back);
        assertEquals(e.distance, back.distance, 0.01);
        assertEquals(e.velocity, back.velocity, 0.1);
        assertEquals(e.x, back.x, 0.01);
        assertEquals(e.heading, back.heading, 1e-3);
        assertEquals("LEFT", back.hive);
        assertNull(ShotTable.parse("distance_in,velocity_ticks_per_s,x,y,heading_deg,alliance,hive"));
    }
}
