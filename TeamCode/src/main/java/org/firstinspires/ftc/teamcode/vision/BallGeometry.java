package org.firstinspires.ftc.teamcode.vision;

/**
 * Pinhole model of the ball camera: where a ball lying on the tiles shows up in the frame, how big, and back from a
 * pixel to where the ball is relative to the robot. Plain Java, shared by the pipeline (size gate) and the robot side
 * (chasing).
 *
 * Robot frame: inches from the robot center on the tiles, +x forward, +y left, +z up.
 * Image: pixels from the top-left corner of the (undistorted) frame, +u right, +v down.
 * Positions are of the ball's center, which sits diameter / 2 above the tiles.
 */
public class BallGeometry {
    /** Where the camera sits on the robot and which way it looks. */
    public static class Mount {
        public final double forward, left, height; // lens position, robot frame, inches
        public final double yaw;                   // radians, + = turned left (CCW seen from above)
        public final double tilt;                  // radians below horizontal

        public Mount(double forward, double left, double height, double yaw, double tilt) {
            this.forward = forward;
            this.left = left;
            this.height = height;
            this.yaw = yaw;
            this.tilt = tilt;
        }
    }

    public final int width, height;  // frame size, pixels
    public final double fx, fy, cx, cy;
    public final Mount mount;

    // Camera axes in the robot frame: optical axis (forward), image right, image down
    private final double fwdX, fwdY, fwdZ, rightX, rightY, downX, downY, downZ;

    public BallGeometry(int width, int height, double fx, double fy, double cx, double cy, Mount mount) {
        this.width = width;
        this.height = height;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.mount = mount;
        double cosT = Math.cos(mount.tilt), sinT = Math.sin(mount.tilt);
        double cosY = Math.cos(mount.yaw), sinY = Math.sin(mount.yaw);
        fwdX = cosT * cosY;
        fwdY = cosT * sinY;
        fwdZ = -sinT;
        rightX = sinY;
        rightY = -cosY;
        downX = -sinT * cosY;
        downY = -sinT * sinY;
        downZ = -cosT;
    }

    /** Uncalibrated: square pixels, principal point in the middle, focal length from the horizontal field of view. */
    public static BallGeometry fromFov(int width, int height, double horizontalFov, Mount mount) {
        double f = (width / 2.0) / Math.tan(horizontalFov / 2);
        return new BallGeometry(width, height, f, f, width / 2.0, height / 2.0, mount);
    }

    /**
     * From a calibration solved at another resolution of the same aspect ratio (calibrate_camera.py solves at the
     * native 1280x800).
     */
    public static BallGeometry fromCalibration(int width, int height, int calibrationWidth, int calibrationHeight,
                                               double fx, double fy, double cx, double cy, Mount mount) {
        double sx = width / (double) calibrationWidth, sy = height / (double) calibrationHeight;
        if (Math.abs(sx - sy) > 0.01 * Math.max(sx, sy)) {
            throw new IllegalArgumentException(String.format("%dx%d frames aren't a resize of the %dx%d calibration",
                    width, height, calibrationWidth, calibrationHeight));
        }
        return new BallGeometry(width, height, fx * sx, fy * sy, cx * sx, cy * sy, mount);
    }

    /**
     * Distance along the optical axis from the lens to a ball of this diameter whose center is at pixel (u, v), or NaN
     * if no ball on the tiles can be there (at or above the horizon).
     */
    public double depth(double u, double v, double diameter) {
        double a = (u - cx) / fx, b = (v - cy) / fy;
        double rayZ = b * downZ + fwdZ; // the ray's vertical component (its optical-axis component is 1)
        double drop = diameter / 2 - mount.height;
        if (rayZ >= -1e-6 || drop >= 0) return Double.NaN;
        return drop / rayZ;
    }

    /** Pixel diameter a ball of this diameter centered at (u, v) should have, or NaN if it can't be there. */
    public double expectedDiameterPx(double u, double v, double diameter) {
        return fy * diameter / depth(u, v, diameter);
    }

    /**
     * Ball center in the robot frame, {x, y}, for a ball of this diameter centered at pixel (u, v), or null if no ball
     * on the tiles can be there.
     */
    public double[] robotPosition(double u, double v, double diameter) {
        double t = depth(u, v, diameter);
        if (Double.isNaN(t)) return null;
        double a = (u - cx) / fx, b = (v - cy) / fy;
        return new double[]{
                mount.forward + t * (a * rightX + b * downX + fwdX),
                mount.left + t * (a * rightY + b * downY + fwdY)
        };
    }

    /** Pixel {u, v} of a point in the robot frame, or null if it's behind the camera. */
    public double[] project(double x, double y, double z) {
        double dx = x - mount.forward, dy = y - mount.left, dz = z - mount.height;
        double depth = dx * fwdX + dy * fwdY + dz * fwdZ;
        if (depth <= 1e-6) return null;
        double right = dx * rightX + dy * rightY;
        double down = dx * downX + dy * downY + dz * downZ;
        return new double[]{cx + fx * right / depth, cy + fy * down / depth};
    }

    /** Image row of the horizon: a ball on the tiles always appears below it. */
    public double horizonRow() {
        return cy - fy * Math.tan(mount.tilt);
    }

    /**
     * First image row where the center of a ball of this diameter can be, counting only balls closer than maxDepth
     * inches (along the optical axis). Rows above it can be cleared before looking for balls.
     */
    public double firstFloorRow(double diameter, double maxDepth) {
        // depth = (height - d/2) / (b cos(tilt) + sin(tilt)) <= maxDepth, solved for b = (v - cy) / fy
        double b = ((mount.height - diameter / 2) / maxDepth - Math.sin(mount.tilt)) / Math.cos(mount.tilt);
        return cy + fy * b;
    }
}
