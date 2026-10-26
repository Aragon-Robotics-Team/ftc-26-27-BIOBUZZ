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
        public final double roll;                  // radians about the optical axis, + = the image's right side looks higher

        public Mount(double forward, double left, double height, double yaw, double tilt, double roll) {
            this.forward = forward;
            this.left = left;
            this.height = height;
            this.yaw = yaw;
            this.tilt = tilt;
            this.roll = roll;
        }
    }

    public final int width, height;  // frame size, pixels
    public final double fx, fy, cx, cy;
    public final Mount mount;

    // Camera axes in the robot frame: optical axis (forward), image right, image down
    private final double fwdX, fwdY, fwdZ, rightX, rightY, rightZ, downX, downY, downZ;

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
        // Level camera first (image right horizontal), then rolled about the optical axis
        double levelDownX = -sinT * cosY, levelDownY = -sinT * sinY, levelDownZ = -cosT;
        double cosR = Math.cos(mount.roll), sinR = Math.sin(mount.roll);
        rightX = cosR * sinY - sinR * levelDownX;
        rightY = -cosR * cosY - sinR * levelDownY;
        rightZ = -sinR * levelDownZ;
        downX = sinR * sinY + cosR * levelDownX;
        downY = -sinR * cosY + cosR * levelDownY;
        downZ = cosR * levelDownZ;
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
        double rayZ = a * rightZ + b * downZ + fwdZ; // the ray's vertical component (its optical-axis component is 1)
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
        double right = dx * rightX + dy * rightY + dz * rightZ;
        double down = dx * downX + dy * downY + dz * downZ;
        return new double[]{cx + fx * right / depth, cy + fy * down / depth};
    }

    /**
     * Image row of the horizon, at its highest across the frame (it slopes when the camera is rolled): a ball on the
     * tiles always appears below it.
     */
    public double horizonRow() {
        return rowWhereRayDrops(0);
    }

    /**
     * First image row where the center of a ball of this diameter can be, counting only balls closer than maxDepth
     * inches (along the optical axis). Rows above it can be cleared before looking for balls.
     */
    public double firstFloorRow(double diameter, double maxDepth) {
        // depth = (height - d/2) / -rayZ <= maxDepth
        return rowWhereRayDrops((mount.height - diameter / 2) / maxDepth);
    }

    /** Topmost row, over the frame's width, where the ray drops at least k per unit along the optical axis. */
    private double rowWhereRayDrops(double k) {
        // a * rightZ + b * downZ + fwdZ <= -k, solved for b = (v - cy) / fy; downZ < 0. Linear in a: an edge is highest
        double a = rightZ > 0 ? -cx / fx : (width - cx) / fx;
        double b = (k + fwdZ + a * rightZ) / -downZ;
        return cy + fy * b;
    }
}
