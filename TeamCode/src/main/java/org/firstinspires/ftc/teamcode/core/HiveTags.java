package org.firstinspires.ftc.teamcode.core;

/**
 * Where each hive's AprilTags are, for fusing the Limelight with odometry (PoseFilter). The tags ride on the hive's
 * arm, so none of them has a fixed spot: each one swings on a circle about the arm's pivot, and where it is on that
 * circle is the arm's angle. The angle at rest isn't exact either (the shock absorbers don't always bottom out), so
 * PoseFilter estimates each hive's arm angle along with the robot's pose. Everything else here is fixed.
 *
 * Geometry (Competition Manual 9.6 / 9.9; the sim's SimHives has the same numbers): the arm pivots 43.95 in above the
 * tiles, 12.75 in from the field center toward its own alliance's wall, about an axis along Pedro x (cells along y).
 * Each cell has four 3.25 in tags in a row across the underside of its floor, centered 14.27 in out along the arm
 * from the pivot and 1.6 in below the arm's axis, 3.75 in apart.
 *
 * Frame: Pedro coordinates (inches) plus height above the tiles. Arm angle: radians about +x, so positive raises the
 * cell toward the rear wall (+y); it rests near ±30°.
 */
public final class HiveTags {
    public static final double PIVOT_HEIGHT = 43.95;
    public static final double PIVOT_FROM_CENTER = 12.75;
    public static final double REST_ANGLE = Math.toRadians(30);
    public static final double ALONG_ARM = 14.27;
    public static final double BELOW_ARM = 1.6;
    public static final double TAG_PITCH = 3.25 + 0.5;

    /** Which hive: 0 red, 1 blue. */
    public static final int RED = 0, BLUE = 1;

    private HiveTags() {}

    /** The hive a tag is on (RED or BLUE), or -1 if it isn't a hive tag. Red 30-37, blue 38-45. */
    public static int hiveOf(int id) {
        if (id >= 30 && id <= 37) return RED;
        if (id >= 38 && id <= 45) return BLUE;
        return -1;
    }

    /**
     * Where a tag is with its hive's arm at an angle, Pedro inches: {x, y, height}. Red: 30-33 on the rear cell (+y),
     * 34-37 the audience one. Blue is red's hive turned 180° about the field center: 38-41 on its audience cell,
     * 42-45 its rear one, each row in the turned order.
     */
    public static double[] position(int id, double armAngle) {
        int hive = hiveOf(id);
        if (hive < 0) throw new IllegalArgumentException("not a hive tag: " + id);
        int first = hive == RED ? 30 : 38;
        int i = (id - first) % 4;
        boolean firstCell = id - first < 4;                 // 30-33 / 38-41
        boolean rear = hive == RED ? firstCell : !firstCell; // the cell toward +y
        double across = (i - 1.5) * TAG_PITCH * (hive == RED ? 1 : -1);
        double along = rear ? ALONG_ARM : -ALONG_ARM;       // out along the arm toward +y or -y
        // Turn (along, -BELOW_ARM) about the pivot by the arm angle
        double c = Math.cos(armAngle), s = Math.sin(armAngle);
        double y = along * c + BELOW_ARM * s;
        double z = along * s - BELOW_ARM * c;
        return new double[] {pivotX(hive) + across, 72 + y, PIVOT_HEIGHT + z};
    }

    /** The pivot's Pedro x: toward each alliance's own wall from the center. */
    public static double pivotX(int hive) {
        return hive == RED ? 72 - PIVOT_FROM_CENTER : 72 + PIVOT_FROM_CENTER;
    }

    /** The arm's rest angle with an alliance's cell up, as that alliance sees it (Hive): LEFT is red's rear cell. */
    public static double restAngle(Alliance alliance, Hive up) {
        boolean rearUp = (up == Hive.LEFT) == (alliance == Alliance.RED);
        return rearUp ? REST_ANGLE : -REST_ANGLE;
    }

    /** Which cell is up at an arm angle, as the alliance sees it. */
    public static Hive up(Alliance alliance, double armAngle) {
        boolean rearUp = armAngle > 0;
        return rearUp == (alliance == Alliance.RED) ? Hive.LEFT : Hive.RIGHT;
    }
}
