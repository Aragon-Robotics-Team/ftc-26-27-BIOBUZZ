package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.RobotConstants.Gate.AUTO_SHOOT_MS;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Led;
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
    public final Led led;

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
        led = new Led(hardwareMap, launcher);

        // Launcher before Led, so the light shows this loop's launcher state
        subsystems = new Subsystem[]{drivebase, intake, gate, launcher, led};
    }

    // ---- Hive ----

    /** Center of the opening of our hive's up cell. */
    public Pose hiveTarget() {
        return MatchState.hive.cell(MatchState.alliance);
    }

    public double distanceToHive() {
        return launcher.distanceTo(drivebase.getPose(), hiveTarget());
    }

    public Command setHive(Hive side) {
        return parallel(instant(() -> MatchState.hive = side), led.flash());
    }

    /** After a tip, the other cell is up. */
    public Command flipHive() {
        return parallel(instant(() -> MatchState.hive = MatchState.hive.flipped()), led.flash());
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

    /** For Auto: wait until ready (max 1.5 s so a slow flywheel can't stall), then empty the robot. */
    public Command shoot() {
        return sequential(
                waitUntil(launcher::isReady).raceWith(waitMs(1500)),
                startShooting(),
                waitMs(AUTO_SHOOT_MS),
                stopShooting()
        );
    }

    // ---- Localization ----

    /** Snap the pose to a known spot, e.g. with the robot pushed into a corner. Takes a red-side pose. */
    public Command resetPose(Pose redPose) {
        return parallel(instant(() -> drivebase.setPose(MatchState.alliance.apply(redPose))), led.flash());
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
