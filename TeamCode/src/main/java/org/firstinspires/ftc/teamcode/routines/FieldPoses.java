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

    /** Robot starting at far corner of red alliance*/
    public final Pose farStart;
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
    /** Position to exit after pollen from flower is knocked out. **/
    public final Pose flowerLaunch;
    public final Pose flowerLaunchHeading;
    /** At the rear-wall flower, where the intake picks up its pollen. */
    public final Pose flowerIntake;
    /** Backed off the rear-wall flower, where the flower pollen is shot. */
    public final Pose flowerShoot;
    /** At the alliance-wall flower: flowerIntake's spot turned 90° about its flower, the same place relative to it. */
    public final Pose wallFlowerIntake;

    /**
     * Where to shoot into each cell from, out in front of its opening: startLaunch for the RIGHT (audience) cell, and
     * that spot mirrored across the hive's middle (y = 72 on red) for the LEFT (rear) one. Checked in the sim
     * (ChaseAndShootSimTest, CameraCyclesSimTest).
     */
    public final Pose rightLaunch, leftLaunch;
    /** Out from our garden corner, turned so the ball camera looks along the garden's row of pollen. */
    public final Pose gardenLook;

    /**
     * Knock out the balls from flowers
     * */
    /** Adjust so don't hit flower. */
    public final Pose redLeftFlowerApproach;
    /** Go in to flower by red alliance side. */
    public final Pose redLeftFlowerIn;
    /** Back out slightly of flower by red alliance side. */
    public final Pose redLeftFlowerOut;
    /** Backed off the alliance wall. */
    public final Pose redLeftFlowerBackout;
    /** Approach far flower. */
    public final Pose redFarFlowerApproach;
    public final Pose redFarFlowerAdjust;
    /** Go in to far flower by red alliance side. */
    public final Pose redFarFlowerIn;
    /** Back out slightly of far flower by red alliance side. */
    public final Pose redFarFlowerOut;
    /** Backed off the far wall. */
    public final Pose redFarFlowerBackout;

    /** Where to shoot into a cell from (rightLaunch / leftLaunch). */
    public Pose launch(Hive cell) {
        return cell == Hive.RIGHT ? rightLaunch : leftLaunch;
    }

    public FieldPoses(PoseFactory poseFactory) {
        // Back against the audience wall. The poses down to topPark were tuned on the robot (Oct 30) from a start of
        // y = 5.0366, 4.2134 in short of where the robot really sits, so they're all 4.2134 in up from what was tuned:
        // the same moves on the field
        start = poseFactory.of(56.8946, RobotConstants.Field.WALL + RobotConstants.Field.ROBOT_LENGTH / 2, 90);
        farStart = poseFactory.of(12.5, 139.7, -90);
        startLaunch = poseFactory.of(59.808, 15.1732, 90);
        gardenBorder = poseFactory.of(9.577, 28.2910, -90);
        gardenIntake = poseFactory.of(9.561, 13.2993, -90);
        gardenIntakeBack = poseFactory.of(9.561, 25.2993, -90);
        adjustToPark = poseFactory.of(38.561, 24.2993, -90);
        bottomPark = poseFactory.of(10.5389, 96.1701, 0);
        topPark = poseFactory.of(11.4049, 124.6458, 0);
        gardenCenterLine = poseFactory.of(63.5, 25, 90);
        flowerLaunch = poseFactory.of(63.5, 120, 90);
        flowerLaunchHeading = poseFactory.of(59.808, 125, 0);
        flowerIntake = poseFactory.of(51, 131, 95);
        flowerShoot = poseFactory.of(56, 121, 95);
        Pose rear = RobotConstants.Field.FLOWERS[0], wall = RobotConstants.Field.FLOWERS[2];
        wallFlowerIntake = poseFactory.of(wall.x() + (rear.y() - 131), wall.y() + (51 - rear.x()), 95 + 90);
        rightLaunch = poseFactory.of(56, 20.219975, 90);
        leftLaunch = poseFactory.of(56, 144 - 20.219975, -90);
        // The camera looks YAW left of the intake (RobotConstants.BallVision), so the robot turns that much less than
        // the 210° the camera should look
        gardenLook = poseFactory.of(30, 16, 210 - Math.toDegrees(RobotConstants.BallVision.YAW));
        redLeftFlowerApproach = poseFactory.of(23, 60.8, 180);
        redLeftFlowerIn = poseFactory.of(12.1, 60.2, 180);
        redLeftFlowerOut = poseFactory.of(12.1, 62.2, 180);
        redLeftFlowerBackout = poseFactory.of(20.3, 60.8, 180);
        redFarFlowerApproach = poseFactory.of(48, 110, 90);
        redFarFlowerAdjust = poseFactory.of(62, 115, 90);
        redFarFlowerIn = poseFactory.of(58, 135, 90);
        redFarFlowerOut = poseFactory.of(62.1, 135, 90);
        redFarFlowerBackout = poseFactory.of(66.1, 121.8, 90);
    }
}
