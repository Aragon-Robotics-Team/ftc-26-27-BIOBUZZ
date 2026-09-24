package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public enum Alliance {
    RED,
    BLUE;

    /** Field poses are written for red. Blue gets the mirror image. */
    public Pose apply(Pose redPose) {
        if (this == RED) return redPose;
        // TODO: check against this season's field symmetry. This mirrors across the line x = 72.
        return new Pose(144 - redPose.x(), redPose.y(), Math.PI - redPose.heading());
    }
}
