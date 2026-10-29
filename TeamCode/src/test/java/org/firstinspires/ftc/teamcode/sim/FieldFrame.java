package org.firstinspires.ftc.teamcode.sim;

import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

/**
 * Pedro coordinates to the frame AdvantageScope uses for FTC fields ("center/rotated"), which is also the field frame
 * the sim's game pieces live in.
 *
 * Pedro: inches from the field corner, red alliance wall at x = 0, +y to the red drivers' left, heading CCW from +x.
 * Field: origin at the field center, +X to the right as seen from the red wall, +Y away from it, heading CCW from +X.
 * Inches here; the pose methods return meters, as AdvantageScope expects.
 */
final class FieldFrame {
    static final double METERS_PER_INCH = 0.0254;

    private FieldFrame() {}

    static double x(double pedroX, double pedroY) {
        return 72 - pedroY;
    }

    static double y(double pedroX, double pedroY) {
        return pedroX - 72;
    }

    static double heading(double pedroHeading) {
        return pedroHeading + Math.PI / 2;
    }

    /** Field X back to Pedro x. */
    static double pedroX(double fieldX, double fieldY) {
        return fieldY + 72;
    }

    static double pedroY(double fieldX, double fieldY) {
        return 72 - fieldX;
    }

    static Pose2d pose2d(double pedroX, double pedroY, double pedroHeading) {
        return new Pose2d(x(pedroX, pedroY) * METERS_PER_INCH, y(pedroX, pedroY) * METERS_PER_INCH,
                new Rotation2d(heading(pedroHeading)));
    }

    /**
     * A 3D pose from a Pedro position, height (inches) and angles (radians). Yaw is a Pedro heading; pitch is WPILib's,
     * positive tilting the nose down; roll turns about the pose's own forward axis.
     */
    static Pose3d pose3d(double pedroX, double pedroY, double z, double roll, double pitch, double pedroYaw) {
        return new Pose3d(new Translation3d(x(pedroX, pedroY) * METERS_PER_INCH, y(pedroX, pedroY) * METERS_PER_INCH,
                z * METERS_PER_INCH), new Rotation3d(roll, pitch, heading(pedroYaw)));
    }

    /** A point in the field frame (inches) as a Translation3d in meters. */
    static Translation3d point(double fieldX, double fieldY, double fieldZ) {
        return new Translation3d(fieldX * METERS_PER_INCH, fieldY * METERS_PER_INCH, fieldZ * METERS_PER_INCH);
    }
}
