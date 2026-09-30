package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Hive;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;

public class HiveReaderTest {
    private static final double LENS = 11, TILT = Math.toRadians(30), THRESHOLD = 42;
    private static final double UP = 49.7, DOWN = 35.5; // tag heights with its cell up / down

    /** A tag straight ahead at this horizontal distance and height, in the camera space of a LENS-high, TILT-up camera. */
    private static HiveReader.Tag tagAt(int id, double ahead, double right, double height) {
        double up = height - LENS;
        double z = ahead * Math.cos(TILT) + up * Math.sin(TILT);
        double y = ahead * Math.sin(TILT) - up * Math.cos(TILT); // +y points down the image
        return new HiveReader.Tag(id, right, y, z);
    }

    @Test
    public void heightRoundTrips() {
        for (double ahead : new double[]{20, 45, 90}) {
            for (double h : new double[]{DOWN, UP, 60}) {
                HiveReader.Tag t = tagAt(30, ahead, 5, h);
                assertEquals(h, HiveReader.height(t.y, t.z, LENS, TILT), 1e-9);
            }
        }
    }

    @Test
    public void tiltComesBackFromAKnownTag() {
        HiveReader.Tag t = tagAt(34, 60, 0, UP);
        assertEquals(TILT, HiveReader.tiltFor(t.y, t.z, LENS, UP), 1e-9);
        assertTrue(Double.isNaN(HiveReader.tiltFor(0, 1, LENS, 500))); // no tilt puts a tag 1 in away that high
    }

    @Test
    public void tagIdsMapToOurCells() {
        assertEquals(Hive.LEFT, HiveReader.cellOf(30, Alliance.RED));
        assertEquals(Hive.LEFT, HiveReader.cellOf(33, Alliance.RED));
        assertEquals(Hive.RIGHT, HiveReader.cellOf(34, Alliance.RED));
        assertEquals(Hive.RIGHT, HiveReader.cellOf(37, Alliance.RED));
        assertNull(HiveReader.cellOf(38, Alliance.RED));
        assertEquals(Hive.LEFT, HiveReader.cellOf(38, Alliance.BLUE));
        assertEquals(Hive.RIGHT, HiveReader.cellOf(45, Alliance.BLUE));
        assertNull(HiveReader.cellOf(34, Alliance.BLUE));
        assertNull(HiveReader.cellOf(29, Alliance.RED));
    }

    @Test
    public void highTagMeansItsCellIsUp() {
        HiveReader.Frame f = HiveReader.read(Collections.singletonList(tagAt(34, 50, 0, UP)), Alliance.RED, LENS, TILT, THRESHOLD);
        assertEquals(Hive.RIGHT, f.up);
        assertFalse(f.conflict);
    }

    @Test
    public void lowTagMeansTheOtherCellIsUp() {
        HiveReader.Frame f = HiveReader.read(Collections.singletonList(tagAt(34, 70, 0, DOWN)), Alliance.RED, LENS, TILT, THRESHOLD);
        assertEquals(Hive.LEFT, f.up);
    }

    @Test
    public void bothCellsAgree() {
        // up cell's tags high and near, down cell's tags low and far: one state
        HiveReader.Frame f = HiveReader.read(Arrays.asList(tagAt(34, 40, 3, UP), tagAt(35, 40, -3, UP), tagAt(30, 65, 0, DOWN)),
                Alliance.RED, LENS, TILT, THRESHOLD);
        assertEquals(Hive.RIGHT, f.up);
        assertEquals(3, f.readings.size());
        assertFalse(f.conflict);
    }

    @Test
    public void opponentTagsAreIgnored() {
        HiveReader.Frame f = HiveReader.read(Collections.singletonList(tagAt(42, 50, 0, UP)), Alliance.RED, LENS, TILT, THRESHOLD);
        assertNull(f.up);
        assertTrue(f.readings.isEmpty());
        assertFalse(f.conflict);
    }

    @Test
    public void disagreeingTagsGiveNoState() {
        HiveReader.Frame f = HiveReader.read(Arrays.asList(tagAt(34, 50, 0, UP), tagAt(30, 50, 0, UP)),
                Alliance.RED, LENS, TILT, THRESHOLD);
        assertNull(f.up);
        assertTrue(f.conflict);
    }

    @Test
    public void blueReadsItsOwnTags() {
        HiveReader.Frame f = HiveReader.read(Collections.singletonList(tagAt(42, 50, 0, UP)), Alliance.BLUE, LENS, TILT, THRESHOLD);
        assertEquals(Hive.RIGHT, f.up);
    }
}
