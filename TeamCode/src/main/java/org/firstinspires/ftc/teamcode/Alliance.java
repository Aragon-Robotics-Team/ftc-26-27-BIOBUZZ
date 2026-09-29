package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

public enum Alliance {
    RED,
    BLUE;

    /**
     * Field poses are written for red. The field looks the same from both alliance stations,
     * so blue gets the red pose rotated 180° around the field center.
     */
    public Pose apply(Pose redPose) {
        if (this == RED) return redPose;
        return new Pose(144 - redPose.x(), 144 - redPose.y(), Angle.normalize(redPose.heading() + Math.PI));
    }
}
