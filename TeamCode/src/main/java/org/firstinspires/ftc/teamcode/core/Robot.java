package org.firstinspires.ftc.teamcode.core;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Gate.AUTO_SHOOT_MS;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BallVision;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.HiveVision;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
// import org.firstinspires.ftc.teamcode.subsystems.Led; // LED: commented out until it's on the robot
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.List;

/**
 * Owns every subsystem so TeleOp and Auto build the robot the same way.
 * Commands that coordinate several subsystems (shooting, aiming at the hive) live here.
 */
public class Robot {
    public final Drivetrain drivebase;
    public final Intake intake;
    public final Gate gate;
    public final Launcher launcher;
    // public final Led led; // LED: commented out until it's on the robot
    public final HiveVision hiveVision;
    public final BallVision ballVision;

    private final List<LynxModule> hubs;
    private final Subsystem[] subsystems;

    public Robot(HardwareMap hardwareMap) {
        // Bulk reads: every encoder / velocity read in a loop comes from one read per hub, refreshed in update()
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drivebase = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        launcher = new Launcher(hardwareMap);
        // led = new Led(hardwareMap, launcher); // LED: commented out until it's on the robot
        hiveVision = new HiveVision(hardwareMap); // off until an OpMode enables it
        ballVision = new BallVision(hardwareMap, drivebase::getPose); // off until an OpMode enables it

        // Hive vision first, so this loop aims at the cell it just read; ball vision after the drivetrain, so it has
        // this loop's pose; launcher before Led, so the light shows this loop's launcher state
        subsystems = new Subsystem[]{hiveVision, drivebase, ballVision, intake, gate, launcher /*, led */};
    }

    // ---- Hive ----

    /** Center of the opening of our hive's up cell. */
    public Pose hiveTarget() {
        return MatchState.hive.cell(MatchState.alliance);
    }

    public double distanceToHive() {
        return launcher.distanceTo(drivebase.getPose(), hiveTarget());
    }

    /** The operator says which cell is up. Hive vision gives a hand-set state a moment before it overrules it. */
    public Command setHive(Hive side) {
        return parallel(instant(() -> {
            MatchState.hive = side;
            hiveVision.manualSet();
        }) /*, led.flash() */);
    }

    /** After a tip, the other cell is up. */
    public Command flipHive() {
        return parallel(instant(() -> MatchState.hive = MatchState.hive.flipped()) /*, led.flash() */);
    }

    /** Keep the turret on the hive's up cell, leading it while the robot moves. Runs until interrupted. */
    public Command aimAtHive() {
        return launcher.aimAt(drivebase::getPose, drivebase::getVelocity, this::hiveTarget);
    }

    /** Flywheel speed follows the simulated shot into the hive's up cell. Runs until launcher.idle(). */
    public Command enableFlywheel() {
        return launcher.enableFlywheel(drivebase::getPose, drivebase::getVelocity, this::hiveTarget);
    }

    // ---- Shooting ----

    /** Open the gate and run the intake: everything stored goes to the shooter. */
    public Command startShooting() {
        return parallel(gate.open(), intake.feed());
    }

    /** Close the gate and put the intake back in the driver's mode. */
    public Command stopShooting() {
        return parallel(gate.close(), intake.stopFeeding());
    }

    /**
     * For Auto: wait until ready (max 1.5 s so a slow flywheel can't stall), then empty the robot.
     * Keeps the balls if there's no shot from here.
     */
    public Command shoot() {
        return sequential(
                waitUntil(launcher::isReady).raceWith(waitMs(1500)),
                conditional(launcher::hasShot,
                        sequential(startShooting(), waitMs(AUTO_SHOOT_MS), stopShooting()),
                        instant(() -> {}))
        );
    }

    // ---- Pollen ----

    /**
     * Intake pollen the ball camera sees until there are maxBalls more in the robot, nothing is in view, or timeoutS
     * runs out (see {@link PollenChase}). Needs ballVision enabled during init.
     */
    public Command chasePollen(int maxBalls, double timeoutS) {
        return new PollenChase(this, maxBalls, timeoutS).command();
    }

    // ---- Localization ----

    /** Snap the pose to a known spot, e.g. with the robot pushed into a corner. Takes a red-side pose. */
    public Command resetPose(Pose redPose) {
        return parallel(instant(() -> drivebase.setPose(MatchState.alliance.apply(redPose))) /*, led.flash() */);
    }

    public void start() {
        for (Subsystem subsystem : subsystems) subsystem.start();
    }

    /** Refreshes the bulk-read cache, then updates every subsystem. Call once at the top of each loop. */
    public void update() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        for (Subsystem subsystem : subsystems) subsystem.update();
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) subsystem.stop();
    }

    public void telemetry(Telemetry telemetry) {
        for (Subsystem subsystem : subsystems) subsystem.telemetry(telemetry);
    }
}
