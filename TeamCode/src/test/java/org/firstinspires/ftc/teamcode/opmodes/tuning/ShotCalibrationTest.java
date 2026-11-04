package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.ShotSolver;
import org.firstinspires.ftc.teamcode.core.ShotSolver.Shot;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class ShotCalibrationTest {
    private static final double TRUE_DRAG = 0.8;
    private static final double TRUE_RATIO = 9.0;

    private double savedDrag;
    private double savedMax;

    @Before
    public void save() {
        savedDrag = RobotConstants.Launcher.DRAG_COEFFICIENT;
        savedMax = RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY;
    }

    @After
    public void restore() {
        RobotConstants.Launcher.DRAG_COEFFICIENT = savedDrag;
        RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY = savedMax;
    }

    /** Spots straight out from the red audience cell, with the flywheel speeds a robot with TRUE_DRAG / TRUE_RATIO needs. */
    private static List<ShotCalibration.Sample> realShots() {
        RobotConstants.Launcher.DRAG_COEFFICIENT = TRUE_DRAG;
        RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY = 1e5;
        Pose target = RobotConstants.Field.RED_HIVE_RIGHT_CELL;
        List<ShotCalibration.Sample> samples = new ArrayList<>();
        for (double distance = 36; distance <= 108; distance += 18) {
            // Facing the cell, so the turret pivot is `distance` from it
            Pose robot = new Pose(target.x(), target.y() - distance - RobotConstants.Launcher.TURRET_OFFSET_X,
                    Math.toRadians(90));
            Shot shot = new ShotSolver().solve(robot, Velocity.zero(), target);
            assertTrue("no shot at " + distance, shot.feasible);
            samples.add(new ShotCalibration.Sample(robot, target, shot.distance, TRUE_RATIO * shot.exitSpeed));
        }
        return samples;
    }

    @Test
    public void fitFindsTheDragAndRatio() {
        List<ShotCalibration.Sample> samples = realShots();
        RobotConstants.Launcher.DRAG_COEFFICIENT = savedDrag;
        RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY = savedMax;

        assertTrue(ShotCalibration.canFit(samples));
        ShotCalibration.Result fit = ShotCalibration.fitDrag(samples);
        assertNotNull(fit);
        assertEquals(TRUE_DRAG, fit.drag, 1e-6);
        assertEquals(TRUE_RATIO, fit.meanRatio, 0.05);
        assertTrue("spread " + fit.spread, fit.spread < 0.005);
        // Leaves the constants as it found them
        assertEquals(savedDrag, RobotConstants.Launcher.DRAG_COEFFICIENT, 0);
        assertEquals(savedMax, RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY, 0);
    }

    @Test
    public void tooLittleDragShowsAsRatioRisingWithDistance() {
        List<ShotCalibration.Sample> samples = realShots();
        ShotCalibration.Result result = ShotCalibration.evaluate(samples, 0.3);
        assertTrue("change " + result.changeOverRange, result.changeOverRange > 0.02);
        result = ShotCalibration.evaluate(samples, 1.3);
        assertTrue("change " + result.changeOverRange, result.changeOverRange < -0.02);
    }
}
