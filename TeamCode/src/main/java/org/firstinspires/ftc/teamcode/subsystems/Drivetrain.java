package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Drive.*;

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
import org.firstinspires.ftc.teamcode.pedro.FusedLocalizer;

import java.util.function.DoubleSupplier;

/**
 * Thin wrapper around the Pedro Pathing {@link Follower}, configured in {@link Constants}, localized by the Pinpoint
 * with the hive's AprilTags fused in ({@link FusedLocalizer}).
 * Autos drive paths with {@code PedroCommands.follow(follower, path)}.
 * Twin stick TeleOp driving turns the robot with a PID loop on the localizer heading.
 */
public class Drivetrain implements Subsystem {
    private final Follower follower;
    private final FusedLocalizer localizer;

    private double targetHeading = 0; // field radians, for the front of the robot
    private double turnPower = 0;
    private final ElapsedTime headingTimer = new ElapsedTime();
    private double headingIntegral = 0;
    private double lastHeadingError = 0;
    // Whether the twin stick drive ran last loop / this loop. When it starts, or resumes after a path suspended it,
    // it holds the heading the robot has now instead of turning back to an old target.
    private boolean headingHeld = false;
    private boolean headingHeldThisLoop = false;
    // driveRateSnap: whether it turned onto the drive direction last loop (for the wider hold-on limit)
    private boolean snapping = false;

    public Drivetrain(HardwareMap hardwareMap) {
        // The Pinpoint, with the hive's tags fused in (HiveVision feeds them) when Localization.FUSE_HIVE_TAGS
        localizer = new FusedLocalizer(Constants.localizer(hardwareMap));
        follower = new Follower(localizer, Constants.drivetrain(hardwareMap), Constants.foresight());
    }

    public FusedLocalizer getLocalizer() {
        return localizer;
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

    /**
     * Field-centric drive with rate turning like {@link #driveFieldCentric}, but while the turn stick is centered and
     * the drive stick is pushed hard, a heading within SNAP_ENTER of the drive direction (nose or tail first) is turned
     * onto it. Mecanum loses top speed driving off its own axis: two wheels have to run faster than the other two and
     * hit full power first (about 8% slower at 5° off, 15% at 10°). The turret aims on its own, so the chassis heading
     * is free to change. Runs until interrupted, like driveFieldCentric.
     */
    public Command driveRateSnap(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, double headingOffset) {
        return infinite(() -> {
            double fwd = forward.getAsDouble();
            double left = strafe.getAsDouble();
            double turnCommand = turn.getAsDouble();
            double axis = Math.abs(turnCommand) <= TURN_DEADBAND ? snapAxis(fwd, left, headingOffset) : Double.NaN;
            snapping = !Double.isNaN(axis);
            if (snapping) {
                setTargetHeading(axis);
                turnCommand = turnToward(SNAP_MAX_TURN);
            }
            follower.manual(ManualDrive.fieldCentric(fwd, left, turnCommand, follower.pose().heading(), headingOffset));
        })
                .requiring(this)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND);
    }

    /** Field heading to snap to (the drive direction or its reverse), or NaN to leave the heading to the driver. */
    private double snapAxis(double forward, double left, double headingOffset) {
        // Below this the wheels aren't at full power, so driving off-axis costs no speed
        if (Math.hypot(forward, left) < SNAP_MIN_DRIVE) return Double.NaN;
        double heading = follower.pose().heading();
        double travel = headingOffset + Math.atan2(left, forward); // same frame as ManualDrive.fieldCentric
        // Nose first if the front is within 90° of the drive direction, tail first otherwise
        double axis = Math.abs(Angle.normalizeSigned(travel - heading)) <= Math.PI / 2 ? travel : travel + Math.PI;
        // Wider to stay snapped than to start, so it doesn't flicker at the edge
        double limit = snapping ? SNAP_EXIT : SNAP_ENTER;
        return Math.abs(Angle.normalizeSigned(axis - heading)) < limit ? Angle.normalizeSigned(axis) : Double.NaN;
    }

    /** Whether driveRateSnap is turning the robot onto its drive direction right now. */
    public boolean isSnapping() {
        return snapping && headingHeld;
    }

    /** Turn power (+CCW) toward targetHeading. Call once per loop while holding a heading. */
    private double holdHeading() {
        if (!headingHeld) setTargetHeading(follower.pose().heading());
        return turnToward(HEADING_MAX_POWER);
    }

    /** Heading PID: turn power (+CCW) toward targetHeading, up to maxPower. Call at most once per loop. */
    private double turnToward(double maxPower) {
        double heading = follower.pose().heading();
        double dt = headingTimer.seconds();
        headingTimer.reset();
        if (!headingHeld) {
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
        turnPower = Range.clip(power, -maxPower, maxPower);
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
