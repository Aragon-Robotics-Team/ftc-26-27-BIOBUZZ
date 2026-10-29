package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class FieldRoutesTest {
    @Test
    public void aClearLineNeedsNoWaypoints() {
        assertTrue(FieldRoutes.between(new Pose(20, 20, 0), new Pose(40, 30, 0)).isEmpty());
    }

    @Test
    public void alongTheRearWallItGoesAroundTheFlowerOnTheFieldSide() {
        // From the sim's chase: just past the rear-wall flower's near side, heading for pollen beyond it
        List<double[]> legs = legs(new Pose(34.3, 134.7, 0), new Pose(80, 126, 0));
        assertTrue("it should need a waypoint", legs.size() > 1);
        Pose flower = RobotConstants.Field.FLOWERS[0];
        for (double[] leg : legs) {
            double d = FieldRoutes.distanceToSegment(flower.x(), flower.y(), leg[0], leg[1], leg[2], leg[3]);
            assertTrue("a leg passes " + d + " in from the flower",
                    d >= RobotConstants.Field.FLOWER_RADIUS + FieldRoutes.PASS - 1e-6);
            assertTrue("waypoints stay off the rear wall", leg[3] <= 144 - FieldRoutes.CLEARANCE + 1e-6);
        }
    }

    @Test
    public void aDriveToABallBesideAFlowerIsLeftAlone() {
        Pose flower = RobotConstants.Field.FLOWERS[0];
        assertTrue(FieldRoutes.between(new Pose(48, 100, 0), new Pose(flower.x() + 8, flower.y() - 6, 0)).isEmpty());
    }

    @Test
    public void acrossTheHiveItGoesAroundTheFootInTheWay() {
        // From the blue side of the hive to the RIGHT cell's launch spot
        List<double[]> legs = legs(new Pose(110, 70, 0), new Pose(56, 20.2, 0));
        assertTrue("it should need a waypoint", legs.size() > 1);
        for (double[] leg : legs) {
            for (double[] foot : RobotConstants.Field.HIVE_FEET) {
                double c = FieldRoutes.PASS - 1e-6;
                assertFalse("a leg crosses a hive foot", FieldRoutes.crosses(leg[0], leg[1], leg[2], leg[3],
                        foot[0] - c, foot[1] - c, foot[2] + c, foot[3] + c));
            }
        }
    }

    @Test
    public void fromTheEndOfAFootItGoesOnToTheFarSide() {
        // From the sim's chase: stopped at the near corner of the blue-side foot, pollen on the far side of it
        double[] foot = RobotConstants.Field.HIVE_FEET[1];
        Pose corner = new Pose(foot[0] - FieldRoutes.CLEARANCE, foot[1] - FieldRoutes.CLEARANCE, 0);
        List<double[]> legs = legs(corner, new Pose(112, 66, 0));
        assertTrue("it should need a waypoint", legs.size() > 1);
        double c = FieldRoutes.PASS - 1e-6;
        for (double[] leg : legs) {
            assertFalse("a leg crosses the foot", FieldRoutes.crosses(leg[0], leg[1], leg[2], leg[3],
                    foot[0] - c, foot[1] - c, foot[2] + c, foot[3] + c));
        }
    }

    /** The straight legs of the route from a to b: {x0, y0, x1, y1} each. */
    private static List<double[]> legs(Pose a, Pose b) {
        List<double[]> legs = new ArrayList<>();
        double x = a.x(), y = a.y();
        for (double[] p : FieldRoutes.between(a, b)) {
            legs.add(new double[] {x, y, p[0], p[1]});
            x = p[0];
            y = p[1];
        }
        legs.add(new double[] {x, y, b.x(), b.y()});
        return legs;
    }
}
