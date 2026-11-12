package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.RobotConstants;

/**
 * Named field locations shared by the auto routines. Every routine gets one as {@code poses}, already
 * flipped for its alliance, so tuning a pose here changes it in every auto that uses it.
 *
 * Write poses for the red alliance (inches, degrees). A pose only one routine uses can stay in that routine.
 */
public class FieldPoses {
    /**
     * A spot to line the robot up on at INIT by tile edges alone, before dragging it to its start pose. Walls bend and
     * the cable chain keeps the robot off them, but tile edges don't move: the Pinpoint measures the drag, so the start
     * pose is this spot plus the drag ({@link #afterDrag}).
     */
    public static final class InitSpot {
        public final Pose pose;
        /** How to line up on it, as seen from our driver station (the same words for both alliances). */
        public final String hint;

        InitSpot(Pose pose, String hint) {
            this.pose = pose;
            this.hint = hint;
        }

        /**
         * Where the robot is now: this spot, moved by what the Pinpoint read between lining up here (initRaw) and now
         * (raw). The Pinpoint's own frame doesn't matter, only its move, taken in the robot's frame at the spot.
         */
        public Pose afterDrag(Pose initRaw, Pose raw) {
            double c = Math.cos(initRaw.heading()), s = Math.sin(initRaw.heading());
            double dx = raw.x() - initRaw.x(), dy = raw.y() - initRaw.y();
            double forward = dx * c + dy * s, left = -dx * s + dy * c;
            double turn = Angle.normalize(raw.heading() - initRaw.heading());
            double h = pose.heading();
            return new Pose(pose.x() + forward * Math.cos(h) - left * Math.sin(h),
                    pose.y() + forward * Math.sin(h) + left * Math.cos(h), Angle.normalize(h + turn));
        }
    }

    /** Where the robot is placed at the start of the match. */
    public final Pose start;
    /** Lined up one tile out from the audience wall, then dragged back to {@link #start}. */
    public final InitSpot startInitSpot;
    /** Launch spot just in front of the start, where the preloads are shot. */
    public final Pose startLaunch;
    /** Edge of the garden, where the robot lines up to drive in and intake. */
    public final Pose gardenBorder;
    /** Inside the garden, at the end of the intake drive. */
    public final Pose gardenIntake;
    /** Back up to try again. */
    public final Pose gardenIntakeBack;
    /** After garden intake, adjust position to not hit flower */
    public final Pose adjustToPark;
    /** Bottom side of the loading zone. */
    public final Pose bottomPark;
    /** Top side of the loading zone. **/
    public final Pose topPark;

    /** Lines up to travel through the middle of the hive on the garden side of red. **/
    public final Pose gardenCenterLine;
    /** Flower side launch zone on red. **/
    public final Pose flowerLaunch;
    /** At the rear-wall flower, where the intake picks up its pollen. */
    public final Pose flowerIntake;
    /** Backed off the rear-wall flower, where the flower pollen is shot. */
    public final Pose flowerShoot;

    /**
     * Where to shoot into each cell from, out in front of its opening: startLaunch for the RIGHT (audience) cell, and
     * that spot mirrored across the hive's middle (y = 72 on red) for the LEFT (rear) one. Checked in the sim
     * (ChaseAndShootSimTest, SoloCarrySimTest).
     */
    public final Pose rightLaunch, leftLaunch;
    /** Back against the alliance wall on the second tile from the audience wall (A2), facing the field. */
    public final Pose allianceWallStart;
    /** Lined up one tile out from the alliance wall, then dragged back to (around) {@link #allianceWallStart}. */
    public final InitSpot allianceWallInitSpot;
    /** Out from our garden corner, the ball camera on the garden's row of pollen. */
    public final Pose gardenLook;

    /**
     * The poses from start to flowerShoot were tuned with the robot's side on the tile edge at x = 48 while the code had
     * it at x = 56.8946, this much further out; they're written as tuned and moved back by it, so the robot drives the
     * same moves now that the start pose is where the robot really is (Nov 10).
     */
    private static final double TUNED_X_BIAS = 56.8946 - (48 + 15.748 / 2);



    /** Where to shoot into a cell from (rightLaunch / leftLaunch). */
    public Pose launch(Hive cell) {
        return cell == Hive.RIGHT ? rightLaunch : leftLaunch;
    }

    public FieldPoses(PoseFactory poseFactory) {
        double width = RobotConstants.Field.ROBOT_WIDTH, length = RobotConstants.Field.ROBOT_LENGTH;
        // Back against the audience wall, the side nearest our driver station on the tile edge at x = 48. The poses down
        // to topPark were tuned on the robot (Oct 30) from a start of y = 5.0366, 4.2134 in short of where the robot
        // really sits, so they're all 4.2134 in up from what was tuned: the same moves on the field
        start = poseFactory.of(48 + width / 2, RobotConstants.Field.WALL + length / 2, 90);
        // The same, one tile out: back edge on the tile edge at y = 24
        startInitSpot = new InitSpot(poseFactory.of(48 + width / 2, 24 + length / 2, 90),
                "Facing left, one tile out from the right-hand side wall: back edge on the 1st tile line out from that"
                        + " wall, the side nearest us on the 2nd tile line out from our alliance wall."
                        + " INIT, then drag it straight back to the wall");
        startLaunch = tuned(poseFactory, 59.808, 15.1732, 90);
        gardenBorder = tuned(poseFactory, 9.577, 28.2910, -90);
        gardenIntake = tuned(poseFactory, 9.561, 13.2993, -90);
        gardenIntakeBack = tuned(poseFactory, 9.561, 25.2993, -90);
        adjustToPark = tuned(poseFactory, 38.561, 24.2993, -90);
        bottomPark = tuned(poseFactory, 10.5389, 96.1701, 0);
        topPark = tuned(poseFactory, 11.4049, 124.6458, 0);
        gardenCenterLine = tuned(poseFactory, 60, 40, 90);
        flowerLaunch = tuned(poseFactory, 60, 119.7, 90);
        flowerIntake = tuned(poseFactory, 51, 131, 95);
        flowerShoot = tuned(poseFactory, 56, 121, 95);
        rightLaunch = poseFactory.of(56, 20.219975, 90);
        leftLaunch = poseFactory.of(56, 144 - 20.219975, -90);
        // The camera looks 20° left of the intake (RobotConstants.BallVision.YAW): heading = where to look - 20°
        allianceWallStart = poseFactory.of(RobotConstants.Field.WALL + RobotConstants.Field.ROBOT_LENGTH / 2, 36, 0);
        // Facing the field one tile out from the alliance wall: back edge on x = 24, right side on y = 24
        allianceWallInitSpot = new InitSpot(poseFactory.of(24 + length / 2, 24 + width / 2, 0),
                "Facing away from us, one tile out from our alliance wall: back edge on the 1st tile line out from it,"
                        + " right side on the 1st tile line out from the right-hand side wall."
                        + " INIT, then drag it back to our wall (sliding it along is fine: the drag is measured)");
        gardenLook = poseFactory.of(30, 16, 190);


    }

    /** A pose as tuned on the robot before Nov 10, moved by TUNED_X_BIAS (red x; blue is flipped with the rest). */
    private static Pose tuned(PoseFactory poseFactory, double x, double y, double degrees) {
        return poseFactory.of(x - TUNED_X_BIAS, y, degrees);
    }
}
