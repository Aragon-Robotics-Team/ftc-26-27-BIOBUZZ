package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

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

    FieldPoses(PoseFactory poseFactory) {
        start = poseFactory.of(56, 8, 90);
        startLaunch = poseFactory.of(56, 20.219975, 90);
        gardenBorder = poseFactory.of(28.4, 9.3, 180);
        gardenIntake = poseFactory.of(8.47877, 8.4, 185);
        bottomPark = poseFactory.of(12.2616, 91.7745, 90);
    }
}
