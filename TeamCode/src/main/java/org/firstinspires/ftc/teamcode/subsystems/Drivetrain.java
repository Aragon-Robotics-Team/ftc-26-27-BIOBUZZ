package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Thin wrapper around the Pedro Pathing {@link Follower}, configured in {@link Constants}. */
public class Drivetrain implements Subsystem {
    private final Follower follower;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public Follower getFollower() {
        return follower;
    }

    // ---- Localization ----

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    // ---- TeleOp ----

    /** Robot-centric drive. Inputs are in [-1, 1]. */
    public void drive(double forward, double strafe, double turn) {
        // TODO: field-centric option
        follower.manual(forward, strafe, turn);
    }

    // ---- Autonomous ----

    public void follow(Path path) {
        follower.follow(path);
    }

    public void hold(Pose pose) {
        follower.hold(pose);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public void stop() {
        follower.stop();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Pose", follower.pose());
    }
}
