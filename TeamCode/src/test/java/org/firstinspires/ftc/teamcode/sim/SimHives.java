package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.core.Hive;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.psilynx.psikit.core.wpi.math.Pose3d;

/**
 * The AprilTags on both hives, for the fake Limelight. Positions are in the sim's field frame: Pedro coordinates
 * moved to the field center (inches, +x away from the red wall, +y toward the rear wall), heading CCW from +x.
 *
 * Geometry (Competition Manual 9.6 / 9.9, as in BiobuzzSim's Config.Hive): the arm pivots 43.95 in up, tilted 30°
 * toward the up cell. Each cell has four 3.25 in tag36h11 tags in a row across its floor's underside, centered
 * 14.27 in along the arm from the pivot and 1.6 in below the arm axis (that offset puts them at the 49.7 / 35.5 in
 * HiveReader expects at 30°). The floor tips with the arm, so every tag faces down toward the up cell's side, 60° at
 * 30°. The tags go wherever the arm's angle puts them (BiobuzzRobotSim moves them as a tray turns).
 */
final class SimHives {
    static final double PIVOT_HEIGHT = 43.95;
    static final double HIVE_FROM_CENTER = 12.75; // each pivot from the field center, toward its own alliance
    private static final double ARM_TILT = Math.toRadians(30);
    private static final double TAGS_ALONG_ARM = 14.27;
    private static final double TAGS_BELOW_ARM = 1.6;
    private static final double TAG_SIZE = 3.25;
    private static final double TAG_PITCH = TAG_SIZE + 0.5; // center to center across the cell
    // Red hive pivot, Pedro coordinates: 12.75 in from field center toward the red wall
    private static final double RED_HIVE_X = 72 - HIVE_FROM_CENTER, RED_HIVE_Y = 72;

    private SimHives() {}

    /** Both hives, each with the given cell up as seen from that alliance's own driver station, arms at 30°. */
    static AprilTagLayout layout(Hive redUp, Hive blueUp) {
        boolean redRearUp = redUp == Hive.LEFT, blueRearUp = blueUp == Hive.RIGHT;
        return layout(redRearUp ? ARM_TILT : -ARM_TILT, blueRearUp ? ARM_TILT : -ARM_TILT);
    }

    /** Both hives with their arms at these angles (SimHiveTrays' tray angles: positive raises the rear cell). */
    static AprilTagLayout layout(double redArm, double blueArm) {
        AprilTagLayout.Builder tags = AprilTagLayout.builder();
        // Red: LEFT is the rear cell (+y), 30-33; RIGHT the audience cell, 34-37 (HiveReader.cellOf)
        addHive(tags, redArm, 30, 34, false);
        // Blue sees the field rotated 180° (Alliance.apply): its LEFT is the audience cell, 38-41; RIGHT the rear, 42-45.
        // Worked out as red's, so the arm turns the other way: red's +y cell is blue's audience one
        addHive(tags, -blueArm, 38, 42, true);
        return tags.build();
    }

    /**
     * One hive, worked out as the red one and turned 180° about the field center for blue.
     *
     * @param arm      radians the arm is turned, red frame: positive raises the +y cell
     * @param plusYIds first id on the cell toward +y in the red frame (four in a row); minusYIds likewise
     */
    private static void addHive(AprilTagLayout.Builder tags, double arm, int plusYIds, int minusYIds, boolean blue) {
        double c = Math.cos(arm), sn = Math.sin(arm);
        // The floor's underside faces down, turned with the arm: toward the raised cell's side
        double facing = arm >= 0 ? Math.PI / 2 : -Math.PI / 2;
        double tiltUp = -(Math.PI / 2 - Math.abs(arm));
        for (int side = 1; side >= -1; side -= 2) {
            double along = side * TAGS_ALONG_ARM;
            double y = along * c + TAGS_BELOW_ARM * sn, z = PIVOT_HEIGHT + along * sn - TAGS_BELOW_ARM * c;
            for (int i = 0; i < 4; i++) {
                add(tags, (side > 0 ? plusYIds : minusYIds) + i, RED_HIVE_X + (i - 1.5) * TAG_PITCH, RED_HIVE_Y + y, z,
                        facing, tiltUp, blue);
            }
        }
    }

    /**
     * Where AdvantageScope draws a hive's tray (the "BIOBUZZ Hive Red/Blue" models from
     * tools/advantagescope/build_sim_field.py): on the pivot, turned to the tray's angle (SimHiveTrays: positive
     * raises the rear cell). In AdvantageScope's frame (FieldFrame) the cells lie along x and the arm turns about y,
     * so the angle is the pose's pitch.
     */
    static Pose3d trayPose(boolean blue, double angle) {
        double pivotX = blue ? 144 - RED_HIVE_X : RED_HIVE_X;
        return FieldFrame.pose3d(pivotX, RED_HIVE_Y, PIVOT_HEIGHT, 0, angle, -Math.PI / 2);
    }

    /** Adds a tag given in Pedro red coordinates, turned 180° about the field center for blue. */
    private static void add(AprilTagLayout.Builder tags, int id, double x, double y, double z, double facing,
                            double tiltUp, boolean blue) {
        if (blue) {
            x = 144 - x;
            y = 144 - y;
            facing += Math.PI;
        }
        tags.addTiltedTag(id, x - 72, y - 72, z, facing, tiltUp, TAG_SIZE);
    }
}
