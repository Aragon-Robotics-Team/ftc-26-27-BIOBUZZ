package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static org.firstinspires.ftc.teamcode.RobotConstants.Drive.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

/**
 * Thin wrapper around the Pedro Pathing {@link Follower}, configured in {@link Constants}.
 * Autos drive paths with {@code PedroCommands.follow(follower, path)}.
 * Twin stick TeleOp driving turns the robot with a PID loop on the localizer heading.
 */
public class Drivetrain implements Subsystem {
    private final Follower follower;

    private double targetHeading = 0; // field radians, for the front of the robot
    private double turnPower = 0;
    private final ElapsedTime headingTimer = new ElapsedTime();
    private double headingIntegral = 0;
    private double lastHeadingError = 0;
    // Whether the twin stick drive ran last loop / this loop. When it starts, or resumes after a path suspended it,
    // it holds the heading the robot has now instead of turning back to an old target.
    private boolean headingHeld = false;
    private boolean headingHeldThisLoop = false;

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

    /** Also holds the new heading, so a corner reset doesn't turn the robot to a target from before the reset. */
    public void setPose(Pose pose) {
        follower.setPose(pose);
        setTargetHeading(pose.heading());
    }

    /** Field-relative velocity: inches/sec in x and y, radians/sec CCW. */
    public Velocity getVelocity() {
        return follower.velocity();
    }

    // ---- Heading hold ----

    /** Field heading (radians) the twin stick drive turns the front of the robot to. */
    public double getTargetHeading() {
        return targetHeading;
    }

    public void setTargetHeading(double heading) {
        targetHeading = Angle.normalizeSigned(heading);
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

    /**
     * Field-centric drive like {@link #driveFieldCentric}, but the second stick says which way to face instead of
     * how fast to turn: the intake turns to point where it's pushed, and the robot holds that heading once it's let
     * go. Both sticks use the same frame (forward = away from our driver station, +left).
     *
     * @param faceForward second stick, forward component
     * @param faceLeft    second stick, left component
     */
    public Command driveTwinStick(DoubleSupplier forward, DoubleSupplier strafe,
                                  DoubleSupplier faceForward, DoubleSupplier faceLeft, double headingOffset) {
        return infinite(() -> {
            double x = faceForward.getAsDouble();
            double y = faceLeft.getAsDouble();
            if (Math.hypot(x, y) > HEADING_STICK_THRESHOLD) {
                setTargetHeading(headingOffset + Math.atan2(y, x) - INTAKE_HEADING_OFFSET);
            }
            follower.manual(ManualDrive.fieldCentric(
                    forward.getAsDouble(), strafe.getAsDouble(), holdHeading(),
                    follower.pose().heading(), headingOffset));
        })
                .requiring(this)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    /** Turn power (+CCW) toward targetHeading. Call once per loop while holding a heading. */
    private double holdHeading() {
        double heading = follower.pose().heading();
        double dt = headingTimer.seconds();
        headingTimer.reset();
        if (!headingHeld) {
            setTargetHeading(heading);
            headingIntegral = 0;
            lastHeadingError = 0;
            dt = 0;
        }
        headingHeldThisLoop = true;

        double error = Angle.normalizeSigned(targetHeading - heading);
        // Anti-windup: dump the integral when the error crosses zero
        if (Math.signum(error) != Math.signum(lastHeadingError)) headingIntegral = 0;
        headingIntegral += error * dt;
        lastHeadingError = error;

        // Derivative from the measured turn speed (the target holds still), so a new target doesn't kick the output
        double power = HEADING_KP * error + HEADING_KI * headingIntegral - HEADING_KD * follower.velocity().omega;
        turnPower = Range.clip(power, -HEADING_MAX_POWER, HEADING_MAX_POWER);
        return turnPower;
    }

    @Override
    public void update() {
        headingHeld = headingHeldThisLoop;
        headingHeldThisLoop = false;
        follower.update();
    }

    @Override
    public void stop() {
        follower.stop();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Pose", follower.pose());
        if (headingHeld) {
            telemetry.addData("Heading target / actual (deg)", "%.0f / %.0f",
                    Math.toDegrees(targetHeading), Math.toDegrees(Angle.normalizeSigned(follower.pose().heading())));
            telemetry.addData("Turn power", "%.2f", turnPower);
        }
    }
}
