package org.firstinspires.ftc.teamcode.core;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.vision.HiveReader;

/**
 * Aims from what the turret's Limelight sees, not from the pose alone. The up cell's four tags ride on that cell, so
 * where the camera sees them, against where the pose says they should be, is how far the pose is off for that shot
 * (odometry drift, a heading error, the hive's arm settled short of its stop, the cell not quite where the field
 * constants put it): one field-frame offset, which moves the aim target by the same amount. The pose itself isn't
 * touched, so paths and everything else are as before; only the shot sees the camera.
 *
 * Between frames (the tags out of view, the turret turned away) the last offset is kept: odometry drifts slowly, so it
 * stays good while the robot drives, and the next frame refreshes it. Each cell keeps its own, since an error in where
 * a cell is only shows on that cell. A pose reset clears them both.
 *
 * No hardware, so it's unit tested; HiveVision runs it on the robot.
 */
public final class AimCorrection {
    /** A hive tag in one frame: its id and center in the camera's frame (inches, +x right, +y down, +z out). */
    public static final class Tag {
        final int id;
        final double x, y, z;

        public Tag(int id, double x, double y, double z) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    private final PoseFilter.Camera camera;
    private final double smoothing, maxCorrection, heightTolerance;
    // Per cell (Hive.ordinal()): the field-frame offset to add to the cell's target, whether there is one, and when
    private final double[] dx = new double[2], dy = new double[2], updated = {Double.NaN, Double.NaN};
    private final boolean[] valid = new boolean[2];
    private final int[] tags = new int[2];
    private int rejected;

    /**
     * @param smoothing       how much of each new frame's offset to take (1: only the latest frame)
     * @param maxCorrection   inches: a frame saying the pose is further off than this is a misread, and is skipped
     * @param heightTolerance inches: a tag this far above or below where the up cell's should be means the hive isn't
     *                        in the state we think (it tipped), so the frame is skipped
     */
    public AimCorrection(PoseFilter.Camera camera, double smoothing, double maxCorrection, double heightTolerance) {
        this.camera = camera;
        this.smoothing = smoothing;
        this.maxCorrection = maxCorrection;
        this.heightTolerance = heightTolerance;
    }

    /** The pose changed underneath (a reset): every offset is stale. */
    public void reset() {
        valid[0] = valid[1] = false;
        updated[0] = updated[1] = Double.NaN;
    }

    /**
     * One Limelight frame. Uses only the up cell's tags; the others are ignored.
     *
     * @param time   when it was taken (seconds)
     * @param pose   the robot's pose then
     * @param turret the turret's angle then (radians, robot-relative, CCW positive)
     * @return whether it moved the offset
     */
    public boolean addFrame(double time, Pose pose, double turret, Tag[] frame, Alliance alliance, Hive up) {
        double arm = HiveTags.restAngle(alliance, up);
        double sumX = 0, sumY = 0;
        int n = 0;
        for (Tag tag : frame) {
            if (HiveReader.cellOf(tag.id, alliance) != up) continue;
            double[] seen = toField(pose, turret, tag.x, tag.y, tag.z);
            double[] model = HiveTags.position(tag.id, arm);
            if (Math.abs(seen[2] - model[2]) > heightTolerance) {
                rejected++;
                return false;
            }
            sumX += seen[0] - model[0];
            sumY += seen[1] - model[1];
            n++;
        }
        if (n == 0) return false;
        // The pose is off by the opposite of how far the tags look moved; the target moves with the tags
        double offX = sumX / n, offY = sumY / n;
        if (Math.hypot(offX, offY) > maxCorrection) {
            rejected++;
            return false;
        }
        int c = up.ordinal();
        if (valid[c]) {
            dx[c] += smoothing * (offX - dx[c]);
            dy[c] += smoothing * (offY - dy[c]);
        } else {
            dx[c] = offX;
            dy[c] = offY;
            valid[c] = true;
        }
        tags[c] = n;
        updated[c] = time;
        return true;
    }

    /** The cell's target moved by its offset, or as it is if the camera hasn't seen that cell since the last reset. */
    public Pose apply(Pose target, Hive cell) {
        int c = cell.ordinal();
        if (!valid[c]) return target;
        return new Pose(target.x() + dx[c], target.y() + dy[c], target.heading());
    }

    /**
     * Where a tag is on the field (Pedro inches, plus height above the tiles), from where the camera saw it and the
     * robot's pose and turret angle at the time. The inverse of PoseFilter.measure.
     */
    public double[] toField(Pose pose, double turret, double right, double down, double forward) {
        double h = pose.heading();
        double yaw = h + turret;
        double cyaw = Math.cos(yaw), syaw = Math.sin(yaw);
        double ct = Math.cos(camera.tilt), st = Math.sin(camera.tilt);
        double lensX = pose.x() + camera.turretX * Math.cos(h) - camera.turretY * Math.sin(h) + camera.ahead * cyaw;
        double lensY = pose.y() + camera.turretX * Math.sin(h) + camera.turretY * Math.cos(h) + camera.ahead * syaw;
        // The camera's forward, right and down axes in the field: (forward, right, down) back to (x, y, z)
        return new double[] {
                lensX + forward * cyaw * ct + right * syaw + down * cyaw * st,
                lensY + forward * syaw * ct - right * cyaw + down * syaw * st,
                camera.height + forward * st - down * ct};
    }

    // ---- State ----

    public boolean has(Hive cell) {
        return valid[cell.ordinal()];
    }

    /** The cell's offset, inches (0 without one). */
    public double dx(Hive cell) {
        return valid[cell.ordinal()] ? dx[cell.ordinal()] : 0;
    }

    public double dy(Hive cell) {
        return valid[cell.ordinal()] ? dy[cell.ordinal()] : 0;
    }

    /** When the cell's offset last changed (seconds), NaN if never since the last reset. */
    public double updated(Hive cell) {
        return updated[cell.ordinal()];
    }

    /** Up-cell tags in the frame that last moved the cell's offset. */
    public int tags(Hive cell) {
        return tags[cell.ordinal()];
    }

    /** Frames skipped as misreads or a tipped hive. */
    public int getRejected() {
        return rejected;
    }
}
