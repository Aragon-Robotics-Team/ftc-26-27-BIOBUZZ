package org.firstinspires.ftc.teamcode.routines;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class InitSpotTest {
    private static final double SPOT_X = 55.874, SPOT_Y = 32, SPOT_H = Math.toRadians(90);
    private final FieldPoses.InitSpot spot = new FieldPoses.InitSpot(new Pose(SPOT_X, SPOT_Y, SPOT_H), "");

    /** The Pinpoint's pose after moving forward / left (inches) in its own frame and turning (radians). */
    private static Pose moved(Pose from, double forward, double left, double turn) {
        double c = Math.cos(from.heading()), s = Math.sin(from.heading());
        return new Pose(from.x() + forward * c - left * s, from.y() + forward * s + left * c, from.heading() + turn);
    }

    private static void assertPose(double x, double y, double heading, Pose actual) {
        assertEquals("x", x, actual.x(), 1e-9);
        assertEquals("y", y, actual.y(), 1e-9);
        assertEquals("heading", heading, actual.heading(), 1e-9);
    }

    @Test
    public void notDraggedIsTheSpot() {
        Pose raw = new Pose(3, -7, 1.2);
        assertPose(SPOT_X, SPOT_Y, SPOT_H, spot.afterDrag(raw, raw));
    }

    @Test
    public void draggedStraightBackAnyPinpointFrame() {
        // Whatever the Pinpoint read at the spot, 22.75 in backwards is 22.75 in toward the wall behind the spot
        for (Pose initRaw : new Pose[]{new Pose(0, 0, 0), new Pose(10, -5, Math.toRadians(30)), new Pose(-3, 8, -2.5)}) {
            assertPose(SPOT_X, SPOT_Y - 22.75, SPOT_H, spot.afterDrag(initRaw, moved(initRaw, -22.75, 0, 0)));
        }
    }

    @Test
    public void sidewaysAndTurnedFollowTheRobot() {
        Pose initRaw = new Pose(10, -5, Math.toRadians(30));
        // Facing +y at the spot, so the robot's left is -x
        assertPose(SPOT_X - 3, SPOT_Y, SPOT_H, spot.afterDrag(initRaw, moved(initRaw, 0, 3, 0)));
        assertPose(SPOT_X, SPOT_Y, SPOT_H + Math.toRadians(5), spot.afterDrag(initRaw, moved(initRaw, 0, 0, Math.toRadians(5))));
    }
}
