package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.HiveTags;
import org.firstinspires.ftc.teamcode.core.PoseFilter;
import org.firstinspires.ftc.teamcode.core.RobotConstants;

/**
 * The Pinpoint's odometry with the hive's AprilTags fused in (core.PoseFilter), as a Pedro localizer: the follower,
 * the turret's aim and everything else that asks for the pose get the fused one. The Pinpoint itself is never
 * corrected; each loop its motion since the last one moves the filter's pose, and HiveVision hands the filter what the
 * Limelight sees. With nothing seen it's the Pinpoint's pose, moved by any reset.
 *
 * Pedro's tuners still use the bare Pinpoint (Constants.localizer).
 */
public class FusedLocalizer implements Localizer {
    private final Localizer odometry;
    private final PoseFilter filter;
    private Pose lastRaw;
    private double lastTime = Double.NaN;
    private MotionState state = MotionState.zero();

    public FusedLocalizer(Localizer odometry) {
        this.odometry = odometry;
        lastRaw = odometry.pose();
        filter = new PoseFilter(new PoseFilter.Camera(RobotConstants.Launcher.TURRET_OFFSET_X,
                RobotConstants.Launcher.TURRET_OFFSET_Y, RobotConstants.Vision.LENS_AHEAD,
                RobotConstants.Vision.LENS_HEIGHT, RobotConstants.Vision.TILT),
                lastRaw.x(), lastRaw.y(), lastRaw.heading(),
                // Both hives start a match with their RIGHT cell up
                HiveTags.restAngle(Alliance.RED, Hive.RIGHT), HiveTags.restAngle(Alliance.BLUE, Hive.RIGHT));
        state = MotionState.ofVelocity(lastRaw, Velocity.zero());
    }

    @Override
    public void update() {
        odometry.update();
        double now = now();
        Pose raw = odometry.pose();
        // The Pinpoint's move since last loop, in the robot's frame at the start of it
        double c = Math.cos(lastRaw.heading()), s = Math.sin(lastRaw.heading());
        double dx = raw.x() - lastRaw.x(), dy = raw.y() - lastRaw.y();
        double dh = Math.atan2(Math.sin(raw.heading() - lastRaw.heading()), Math.cos(raw.heading() - lastRaw.heading()));
        filter.predict(now, dx * c + dy * s, -dx * s + dy * c, dh, Double.isNaN(lastTime) ? 0 : now - lastTime);
        lastRaw = raw;
        lastTime = now;
        publish();
    }

    /** The filter's pose, with the Pinpoint's velocity turned by how far the filter has corrected its heading. */
    private void publish() {
        Velocity v = odometry.velocity();
        double turn = filter.heading() - lastRaw.heading();
        double c = Math.cos(turn), s = Math.sin(turn);
        state = MotionState.ofVelocity(new Pose(filter.x(), filter.y(), filter.heading()),
                new Velocity(v.vx * c - v.vy * s, v.vx * s + v.vy * c, v.omega));
    }

    @Override
    public void setPose(Pose pose) {
        odometry.setPose(pose);
        lastRaw = odometry.pose();
        filter.setPose(now(), pose.x(), pose.y(), pose.heading());
        publish();
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void reset() {
        odometry.reset();
    }

    // ---- Tags ----

    /** A hive tag in a Limelight frame taken at time (seconds, System.nanoTime's clock); see PoseFilter.addTag. */
    public boolean addTag(double time, double turret, int id, double x, double y, double z) {
        return filter.addTag(time, turret, id, x, y, z);
    }

    /** The operator (or a new match) says which cell of an alliance's hive is up. */
    public void setHive(Alliance alliance, Hive up) {
        filter.setArm(alliance == Alliance.RED ? HiveTags.RED : HiveTags.BLUE, HiveTags.restAngle(alliance, up));
    }

    /** When update() last read the Pinpoint (seconds, System.nanoTime's clock), NaN before the first. */
    public double getLastUpdateTime() {
        return lastTime;
    }

    public PoseFilter getFilter() {
        return filter;
    }

    /** The Pinpoint's own pose, uncorrected. */
    public Pose getOdometryPose() {
        return lastRaw;
    }

    public static double now() {
        return System.nanoTime() / 1e9;
    }
}
