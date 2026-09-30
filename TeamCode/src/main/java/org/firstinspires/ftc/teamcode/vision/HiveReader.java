package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Hive;

import java.util.ArrayList;
import java.util.List;

/**
 * Reads which of our hive's cells is up from the AprilTags in one camera frame. No hardware, so it's unit tested.
 *
 * Each cell has four tags on the underside of its floor (Competition Manual 9.9). They tip with the hive, so an up
 * cell's tags sit about 49.7 in above the tiles and a down cell's about 35.5 in; the gap is 14.3 in whatever the
 * damper rest angle. The tag ID says which cell it's on, its height says whether that cell is up, and the two cells
 * always move together, so any one of our tags gives the whole hive. Tags only read from their face side, and all
 * eight face the side of the field where the up cell is.
 */
public final class HiveReader {
    private HiveReader() {}

    /** One detected tag: its ID and where it is in the Limelight's camera space, in inches (+x right, +y down, +z out of the lens). */
    public static class Tag {
        public final int id;
        public final double x, y, z;

        public Tag(int id, double x, double y, double z) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    /** One of our tags, read. */
    public static class Reading {
        public final Tag tag;
        /** The cell the tag is on. */
        public final Hive cell;
        /** Inches above the tiles. */
        public final double height;
        /** The hive state this tag says: its own cell if it's high, the other one if it's low. */
        public final Hive up;

        Reading(Tag tag, Hive cell, double height, Hive up) {
            this.tag = tag;
            this.cell = cell;
            this.height = height;
            this.up = up;
        }
    }

    /** What one frame says. */
    public static class Frame {
        /** Our tags in the frame, in detection order. */
        public final List<Reading> readings;
        /** Which cell is up, or null when none of our tags were seen or they disagree. */
        public final Hive up;
        /** True when our tags in the frame disagree about the state. */
        public final boolean conflict;

        Frame(List<Reading> readings, Hive up, boolean conflict) {
            this.readings = readings;
            this.up = up;
            this.conflict = conflict;
        }
    }

    /**
     * Height of a tag above the tiles, from its camera-space position.
     *
     * @param lensHeight inches from the tiles to the lens
     * @param tilt       camera pitch above horizontal, radians (landscape mount, turret axis vertical)
     */
    public static double height(double y, double z, double lensHeight, double tilt) {
        // +z points out of the lens, tilted up; +y points down the image
        return lensHeight + z * Math.sin(tilt) - y * Math.cos(tilt);
    }

    /**
     * Camera tilt that puts a tag at a known height: solves height() for the tilt. NaN if no tilt can.
     * z·sin(t) − y·cos(t) = r·sin(t − b) with r = √(y² + z²) and b = atan2(y, z).
     */
    public static double tiltFor(double y, double z, double lensHeight, double knownHeight) {
        double r = Math.hypot(y, z);
        double s = (knownHeight - lensHeight) / r;
        if (r == 0 || Math.abs(s) > 1) return Double.NaN;
        return Math.atan2(y, z) + Math.asin(s);
    }

    /**
     * Which of our cells a tag is on, as seen from our driver station, or null if it isn't one of ours.
     * Red: 30-33 on the rear (scoring side) cell, 34-37 on the audience cell. Blue: 38-41 audience, 42-45 rear.
     * Blue's LEFT is the audience cell because Alliance.apply rotates red's poses 180°.
     */
    public static Hive cellOf(int id, Alliance alliance) {
        if (alliance == Alliance.RED) {
            if (id >= 30 && id <= 33) return Hive.LEFT;
            if (id >= 34 && id <= 37) return Hive.RIGHT;
        } else {
            if (id >= 38 && id <= 41) return Hive.LEFT;
            if (id >= 42 && id <= 45) return Hive.RIGHT;
        }
        return null;
    }

    /**
     * @param threshold tags higher than this many inches are on the up cell
     */
    public static Frame read(List<Tag> tags, Alliance alliance, double lensHeight, double tilt, double threshold) {
        List<Reading> readings = new ArrayList<>();
        Hive up = null;
        boolean conflict = false;
        for (Tag tag : tags) {
            Hive cell = cellOf(tag.id, alliance);
            if (cell == null) continue;
            double h = height(tag.y, tag.z, lensHeight, tilt);
            Hive says = h > threshold ? cell : cell.flipped();
            readings.add(new Reading(tag, cell, h, says));
            if (up == null) up = says;
            else if (up != says) conflict = true;
        }
        return new Frame(readings, conflict ? null : up, conflict);
    }
}
