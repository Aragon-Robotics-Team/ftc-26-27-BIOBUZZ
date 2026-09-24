package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/** Where the robot starts in Auto. Poses are written for red; blue gets the mirror image. */
public enum StartPosition {
    // TODO: real start spots + poses
    NEAR(Pose.zero()),
    FAR(Pose.zero());

    private final Pose redPose;

    StartPosition(Pose redPose) {
        this.redPose = redPose;
    }

    public Pose pose(Alliance alliance) {
        return alliance.apply(redPose);
    }

    public StartPosition next() {
        return values()[(ordinal() + 1) % values().length];
    }
}
