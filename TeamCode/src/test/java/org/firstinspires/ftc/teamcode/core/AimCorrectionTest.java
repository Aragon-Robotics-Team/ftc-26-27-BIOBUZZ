package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * AimCorrection on made-up frames: PoseFilter.measure says what the turret's Limelight sees of the hive's tags from the
 * robot's true pose, and AimCorrection gets them with an odometry pose that's off. Aiming at the corrected target from
 * the wrong pose has to point the turret where the true pose would.
 */
public class AimCorrectionTest {
    private static final PoseFilter.Camera CAMERA = new PoseFilter.Camera(0, 0, 4.8, 12.2, Math.toRadians(45));
    private static final double REST = HiveTags.restAngle(Alliance.RED, Hive.LEFT);
    private static final PoseFilter VIEW = new PoseFilter(CAMERA, 0, 0, 0, REST, HiveTags.restAngle(Alliance.BLUE, Hive.RIGHT));

    private static AimCorrection correction() {
        return new AimCorrection(CAMERA, 1, 12, 6);
    }

    /** What the camera sees of these tags with the robot at pose and the turret at turret. */
    private static AimCorrection.Tag[] frame(Pose pose, double turret, int... ids) {
        double[] s = {pose.x(), pose.y(), pose.heading(), REST, HiveTags.restAngle(Alliance.BLUE, Hive.RIGHT)};
        List<AimCorrection.Tag> tags = new ArrayList<>();
        for (int id : ids) {
            double[] seen = VIEW.measure(s, turret, id);
            if (seen != null) tags.add(new AimCorrection.Tag(id, seen[0], seen[1], seen[2]));
        }
        return tags.toArray(new AimCorrection.Tag[0]);
    }

    /** Robot-relative turret angle straight at a target from a pose (the turret axis is the robot center here). */
    private static double aimAngle(Pose pose, Pose target) {
        return Angle.normalizeSigned(Math.atan2(target.y() - pose.y(), target.x() - pose.x()) - pose.heading());
    }

    @Test
    public void toFieldUndoesTheCamera() {
        AimCorrection aim = correction();
        Pose[] poses = {new Pose(60, 120, Math.toRadians(-90)), new Pose(40, 110, Math.toRadians(-60)),
                new Pose(75, 125, Math.toRadians(-120))};
        for (Pose pose : poses) {
            for (double turret = -0.4; turret <= 0.4; turret += 0.2) {
                double[] s = {pose.x(), pose.y(), pose.heading(), REST, 0};
                for (int id = 30; id <= 33; id++) {
                    double[] seen = VIEW.measure(s, turret, id);
                    if (seen == null) continue;
                    double[] field = aim.toField(pose, turret, seen[0], seen[1], seen[2]);
                    double[] tag = HiveTags.position(id, REST);
                    for (int i = 0; i < 3; i++) assertEquals(tag[i], field[i], 1e-9);
                }
            }
        }
    }

    @Test
    public void correctedTargetAimsLikeTheTruePose() {
        Pose cell = Hive.LEFT.cell(Alliance.RED);
        Pose truth = new Pose(52, 118, Math.toRadians(-80));
        // Odometry drifted: 3 in one way, 2 the other, and 2.5° of heading
        Pose odometry = new Pose(truth.x() + 3, truth.y() - 2, truth.heading() + Math.toRadians(2.5));
        double turret = aimAngle(truth, cell);

        AimCorrection aim = correction();
        assertTrue(aim.addFrame(0, odometry, turret, frame(truth, turret, 30, 31, 32, 33), Alliance.RED, Hive.LEFT));
        assertEquals(4, aim.tags(Hive.LEFT));

        double wanted = aimAngle(truth, cell);
        double uncorrected = aimAngle(odometry, cell);
        double corrected = aimAngle(odometry, aim.apply(cell, Hive.LEFT));
        System.out.printf("aim error: odometry %.2f°, corrected %.3f°%n",
                Math.toDegrees(uncorrected - wanted), Math.toDegrees(corrected - wanted));
        assertTrue(Math.abs(uncorrected - wanted) > Math.toRadians(4));
        assertEquals(wanted, corrected, Math.toRadians(0.3));
        // And the distance, for the flywheel
        double trueDistance = Math.hypot(cell.x() - truth.x(), cell.y() - truth.y());
        Pose moved = aim.apply(cell, Hive.LEFT);
        assertEquals(trueDistance, Math.hypot(moved.x() - odometry.x(), moved.y() - odometry.y()), 0.5);
    }

    @Test
    public void onlyTheUpCellsTagsCount() {
        Pose pose = new Pose(60, 118, Math.toRadians(-90));
        AimCorrection aim = correction();
        // RIGHT is up, so the rear cell's tags (LEFT) aren't the target's
        assertFalse(aim.addFrame(0, pose, 0, frame(pose, 0, 30, 31, 32, 33), Alliance.RED, Hive.RIGHT));
        assertFalse(aim.has(Hive.RIGHT));
        Pose cell = Hive.RIGHT.cell(Alliance.RED);
        assertEquals(cell.x(), aim.apply(cell, Hive.RIGHT).x(), 0);
    }

    @Test
    public void aPerfectPoseNeedsNoCorrection() {
        Pose pose = new Pose(60, 118, Math.toRadians(-90));
        AimCorrection aim = correction();
        assertTrue(aim.addFrame(0, pose, 0.05, frame(pose, 0.05, 30, 31, 32, 33), Alliance.RED, Hive.LEFT));
        assertEquals(0, aim.dx(Hive.LEFT), 1e-9);
        assertEquals(0, aim.dy(Hive.LEFT), 1e-9);
    }

    @Test
    public void misreadsAndTippedHivesAreSkipped() {
        Pose truth = new Pose(60, 118, Math.toRadians(-90));
        AimCorrection aim = correction();
        // A pose 20 in off is beyond MAX_AIM_CORRECTION: a misread, not a correction
        assertFalse(aim.addFrame(0, new Pose(80, 118, truth.heading()), 0, frame(truth, 0, 30, 31), Alliance.RED, Hive.LEFT));
        // Tags at the down cell's height: the hive isn't in the state we think
        double[] s = {truth.x(), truth.y(), truth.heading(), -REST, 0};
        double[] seen = VIEW.measure(s, 0, 30);
        assertTrue(seen != null);
        AimCorrection.Tag[] tipped = {new AimCorrection.Tag(30, seen[0], seen[1], seen[2])};
        assertFalse(aim.addFrame(0, truth, 0, tipped, Alliance.RED, Hive.LEFT));
        assertFalse(aim.has(Hive.LEFT));
        assertEquals(2, aim.getRejected());
    }

    @Test
    public void resetForgetsTheOffsets() {
        Pose truth = new Pose(60, 118, Math.toRadians(-90));
        AimCorrection aim = correction();
        assertTrue(aim.addFrame(0, new Pose(62, 118, truth.heading()), 0, frame(truth, 0, 30, 31, 32, 33),
                Alliance.RED, Hive.LEFT));
        assertTrue(aim.has(Hive.LEFT));
        aim.reset();
        assertFalse(aim.has(Hive.LEFT));
    }

    @Test
    public void blueMirrorsRed() {
        // Blue's LEFT cell, seen from the mirrored spot, with the same odometry error mirrored
        Pose cell = Hive.LEFT.cell(Alliance.BLUE);
        Pose truth = Alliance.BLUE.apply(new Pose(52, 118, Math.toRadians(-80)));
        Pose odometry = new Pose(truth.x() - 3, truth.y() + 2, truth.heading() + Math.toRadians(2.5));
        double turret = aimAngle(truth, cell);
        double blueRest = HiveTags.restAngle(Alliance.BLUE, Hive.LEFT);
        double[] s = {truth.x(), truth.y(), truth.heading(), 0, blueRest};
        List<AimCorrection.Tag> tags = new ArrayList<>();
        for (int id = 38; id <= 45; id++) {
            double[] seen = VIEW.measure(s, turret, id);
            if (seen != null) tags.add(new AimCorrection.Tag(id, seen[0], seen[1], seen[2]));
        }
        AimCorrection aim = correction();
        assertTrue(aim.addFrame(0, odometry, turret, tags.toArray(new AimCorrection.Tag[0]), Alliance.BLUE, Hive.LEFT));
        assertEquals(aimAngle(truth, cell), aimAngle(odometry, aim.apply(cell, Hive.LEFT)), Math.toRadians(0.3));
    }
}
