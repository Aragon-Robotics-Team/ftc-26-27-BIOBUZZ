package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

/**
 * Thin wrapper around the Pedro Pathing {@link Follower}, configured in {@link Constants}.
 * Autos drive paths with {@code PedroCommands.follow(follower, path)}.
 */
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

    // ---- Commands ----

    /**
     * Field-centric drive from the sticks (inputs in [-1, 1], robot-centric sign convention: +forward, +left, +CCW).
     * Stick forward drives away from the driver station. Runs until interrupted; a later command that requires
     * the drivetrain (e.g. an auto-align path) suspends this, and it resumes after.
     *
     * @param headingOffset radians from the field +x axis to "away from our driver station" (0 red, PI blue)
     */
    public Command driveFieldCentric(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, double headingOffset) {
        return infinite(() -> follower.manual(ManualDrive.fieldCentric(
                forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble(),
                follower.pose().heading(), headingOffset)))
                .requiring(this)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
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
