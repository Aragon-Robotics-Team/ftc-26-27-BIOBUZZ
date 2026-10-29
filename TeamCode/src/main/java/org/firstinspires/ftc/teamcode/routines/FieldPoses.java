package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.RobotConstants;

/**
 * Named field locations shared by the auto routines. Every routine gets one as {@code poses}, already
 * flipped for its alliance, so tuning a pose here changes it in every auto that uses it.
 *
 * Write poses for the red alliance (inches, degrees). A pose only one routine uses can stay in that routine.
 */
public class FieldPoses {
    /** Where the robot is placed at the start of the match. */
    public final Pose start;
    /** Launch spot just in front of the start, where the preloads are shot. */
    public final Pose startLaunch;
    /** Edge of the garden, where the robot lines up to drive in and intake. */
    public final Pose gardenBorder;
    /** Inside the garden, at the end of the intake drive. */
    public final Pose gardenIntake;
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
    /** Out from our garden corner, the ball camera on the garden's row of pollen. */
    public final Pose gardenLook;



    /** Where to shoot into a cell from (rightLaunch / leftLaunch). */
    public Pose launch(Hive cell) {
        return cell == Hive.RIGHT ? rightLaunch : leftLaunch;
    }

    public FieldPoses(PoseFactory poseFactory) {
        start = poseFactory.of(56, 8, 90);
        startLaunch = poseFactory.of(56, 20.219975, 90);
        gardenBorder = poseFactory.of(28.4, 9.3, 180);
        gardenIntake = poseFactory.of(8.47877, 8.4, 185);
        bottomPark = poseFactory.of(12.2616, 91.7745, 90);
        topPark = poseFactory.of(15,120, 0);
        gardenCenterLine = poseFactory.of(60, 40, 90);
        flowerLaunch = poseFactory.of(60, 119.7, 90);
        flowerIntake = poseFactory.of(51, 131, 95);
        flowerShoot = poseFactory.of(56, 121, 95);
        rightLaunch = poseFactory.of(56, 20.219975, 90);
        leftLaunch = poseFactory.of(56, 144 - 20.219975, -90);
        // The camera looks 20° left of the intake (RobotConstants.BallVision.YAW): heading = where to look - 20°
        allianceWallStart = poseFactory.of(RobotConstants.Field.ROBOT_LENGTH / 2, 36, 0);
        gardenLook = poseFactory.of(30, 16, 190);


    }
}
