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
    public final Pose flowerReady;
    /** Position right before pollen from flower is knocked out. **/
    public final Pose flowerKnock;
    /** Position after pollen from flower is knocked out. **/
    public final Pose flowerOut;
    /** Position to exit after pollen from flower is knocked out. **/
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

    // Clearing the flowers (OneCycleGardenParkWithFlowers): slide back and forth along the wall beside each flower to
    // knock its pollen out
    /** Out from the garden toward the alliance-wall flower. Heading is the direction of travel from adjustToPark. */
    public final Pose allianceFlowerApproach;
    /** Against the alliance wall, on the audience side of its flower, facing the wall. */
    public final Pose allianceFlowerLow;
    /** Slid 5 in along the alliance wall toward the flower. */
    public final Pose allianceFlowerHigh;
    /** Backed off the alliance wall, ready to head for the rear. */
    public final Pose allianceFlowerBackOff;
    /** Partway to the rear-wall flower. Heading is the direction of travel from allianceFlowerBackOff. */
    public final Pose rearFlowerApproach;
    /** Against the rear wall, on the alliance-wall side of its flower, facing the wall. */
    public final Pose rearFlowerLeft;
    /** Slid 5 in along the rear wall toward the flower. */
    public final Pose rearFlowerRight;
    /** Backed off the rear wall. */
    public final Pose rearFlowerBackOff;

    /** Where to shoot into a cell from (rightLaunch / leftLaunch). */
    public Pose launch(Hive cell) {
        return cell == Hive.RIGHT ? rightLaunch : leftLaunch;
    }

    public FieldPoses(PoseFactory poseFactory) {
        // Back against the audience wall. The poses down to topPark were tuned on the robot (Oct 30) from a start of
        // y = 5.0366, 4.2134 in short of where the robot really sits, so they're all 4.2134 in up from what was tuned:
        // the same moves on the field
        start = poseFactory.of(56.8946, RobotConstants.Field.WALL + RobotConstants.Field.ROBOT_LENGTH / 2, 90);
        startLaunch = poseFactory.of(59.808, 15.1732, 90);
        gardenBorder = poseFactory.of(9.577, 28.2910, -90);
        gardenIntake = poseFactory.of(9.561, 13.2993, -90);
        gardenIntakeBack = poseFactory.of(9.561, 25.2993, -90);
        adjustToPark = poseFactory.of(38.561, 24.2993, -90);
        bottomPark = poseFactory.of(10.5389, 96.1701, 0);
        topPark = poseFactory.of(11.4049, 124.6458, 0);
        gardenCenterLine = poseFactory.of(60, 40, 90);
        flowerReady = poseFactory.of(10, 60, 180);
        flowerKnock = poseFactory.of(10, 55, 180);
        flowerOut = poseFactory.of(30, 60, 180);
        flowerLaunch = poseFactory.of(60, 119.7, 90);
        flowerIntake = poseFactory.of(51, 131, 95);
        flowerShoot = poseFactory.of(56, 121, 95);
        rightLaunch = poseFactory.of(56, 20.219975, 90);
        leftLaunch = poseFactory.of(56, 144 - 20.219975, -90);
        // The camera looks 20° left of the intake (RobotConstants.BallVision.YAW): heading = where to look - 20°
        allianceWallStart = poseFactory.of(RobotConstants.Field.WALL + RobotConstants.Field.ROBOT_LENGTH / 2, 36, 0);
        gardenLook = poseFactory.of(30, 16, 190);
        allianceFlowerApproach = poseFactory.of(17, 35, 153.6048);
        allianceFlowerLow = poseFactory.of(10, 35, 180);
        allianceFlowerHigh = poseFactory.of(10, 40, 180);
        allianceFlowerBackOff = poseFactory.of(16.3053, 40, 180);
        rearFlowerApproach = poseFactory.of(34, 118.764, 77.3384);
        rearFlowerLeft = poseFactory.of(35, 132, 90);
        rearFlowerRight = poseFactory.of(40, 132, 90);
        rearFlowerBackOff = poseFactory.of(40, 125, 90);    }
}
