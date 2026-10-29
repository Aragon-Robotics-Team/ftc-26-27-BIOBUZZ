package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.PollenChase;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.psilynx.psikit.core.Logger;

/**
 * Carries the alliance alone: as many tips as 30 s allows, with a partner that only starts in our LEFT corner (back to
 * the alliance wall, by the loading zone) and parks. Starts back against the alliance wall on tile A2 (allianceWallStart).
 *
 * The up cell starts with 3 nectar in it (5 pollen's worth), so the 4 preloads tip it. After that it's cycles of the
 * pollen chase on the ball camera, first the garden (gardenLook), then whatever pollen it can see, which after a tip
 * includes what rolled out of the cell that came down; with nothing in sight it tries the next lookout (gardenLook,
 * audienceSpillLook, rearSpillLook). With a hopper full, or nothing more to find, it drives to the launch spot for the
 * cell that's up and shoots.
 *
 * Which cell is up: hive vision, when the Limelight can see the tags (they face the up cell's side), and otherwise a
 * count of what it has put into the up cell: 8 pollen tip a hive (Field Setup Guide 12.3), so once its count reaches
 * that it takes the hive as tipped. It never counts what it can't see go in, so a miss leaves the count high.
 *
 * Leaves itself time to drive to the loading zone before the 30 s are up, whatever it's doing then, and parks at its
 * audience end, so the partner should park at the rear end. It can't take pollen from the flowers.
 */
public class SoloCarry extends AutoRoutine {
    private static final double AUTO_S = 30;
    // Pollen's worth that tips a hive, and what the up cell starts with: 3 nectar, each 5/3 of a pollen (the sim's
    // weights, from 8 pollen tipping a hive and 3 pollen with 3 nectar tipping it)
    private static final double TIP_LOAD = 8, START_LOAD = 5;
    // Time to leave for parking: the drive there at about this speed, plus a margin for the turn and settling
    private static final double PARK_SPEED = 30, PARK_MARGIN_S = 2.5;
    // Time a chase leaves for driving to the launch spot and shooting, and the shortest chase worth starting
    private static final double SHOOT_RESERVE_S = 5.5, MIN_CHASE_S = 2;

    private final ElapsedTime clock = new ElapsedTime();
    private final Pose[] lookouts;
    // The audience end of the loading zone, facing along the wall: the front 2 in into the zone, which leaves the
    // partner the rear 22 in of it (park it there)
    private final Pose park;
    private int nextLookout = 0;
    private int held = RobotConstants.Intake.CAPACITY; // the preload
    private double load = START_LOAD;                 // pollen's worth in the up cell, as far as it knows
    private int tips = 0;
    private String stage = "start";

    public SoloCarry(Robot robot, Alliance alliance) {
        super(robot, alliance);
        lookouts = new Pose[] {poses.gardenLook, poses.audienceSpillLook, poses.rearSpillLook};
        park = poseFactory.of(9, 96 + 2 - RobotConstants.Field.ROBOT_LENGTH / 2, 90);
    }

    @Override
    public Pose startPose() {
        return poses.allianceWallStart;
    }

    @Override
    public boolean usesBallVision() {
        return true;
    }

    @Override
    public void setUp() {
        robot.hiveVision.setEnabled(true);
        clock.reset(); // the match starts now
    }

    @Override
    public Command autoRoutine() {
        Command carry = sequential(
                lazy(this::driveAndShoot), // the preloads
                lazy(() -> sequential(robot.driveTo(lookouts[nextLookout++]), cycle())));
        return sequential(
                deadline(carry.raceWith(waitUntil(this::timeToPark)), robot.aimAtHive(), robot.enableFlywheel()),
                robot.stopShooting(),
                robot.intake.off(),
                instant(() -> stage("parking")),
                robot.driveTo(park),
                instant(() -> stage("parked")));
    }

    /**
     * Chase from here, leaving time to shoot what it gets and park; with nothing found, the next lookout and chase
     * again. A full hopper, or pollen and no time for more, goes to the launch spot first. No time for a chase and
     * nothing to shoot: done, and it parks.
     */
    private Command cycle() {
        double spare = timeLeft() - parkTime() - SHOOT_RESERVE_S;
        if (held == RobotConstants.Intake.CAPACITY || (held > 0 && spare < MIN_CHASE_S)) {
            return sequential(lazy(this::driveAndShoot), lazy(this::cycle));
        }
        if (spare < MIN_CHASE_S) return instant(() -> stage("no time for more"));
        int room = RobotConstants.Intake.CAPACITY - held;
        PollenChase chase = new PollenChase(robot, room, spare);
        stage("chasing for " + room);
        return sequential(
                chase.command(),
                robot.intake.off(),
                lazy(() -> {
                    held += chase.getPickedUp();
                    if (chase.getPickedUp() > 0) return lazy(this::cycle); // keep chasing from here while there's room
                    if (held > 0) return sequential(lazy(this::driveAndShoot), lazy(this::cycle));
                    Pose lookout = lookouts[nextLookout++ % lookouts.length];
                    stage("nothing in sight: to a lookout");
                    return sequential(robot.driveTo(lookout), lazy(this::cycle));
                }));
    }

    /** To the launch spot for the up cell (again if it changes on the way), then everything into it. */
    private Command driveAndShoot() {
        Hive cell = MatchState.hive;
        stage("to the launch spot for " + cell);
        return sequential(
                robot.driveTo(poses.launch(cell)),
                lazy(() -> MatchState.hive != cell ? driveAndShoot() : sequential(
                        instant(() -> stage("shooting " + held + " into " + cell)),
                        waitUntil(robot.launcher::isReady).raceWith(waitMs(1500)),
                        lazy(() -> {
                            if (!robot.launcher.hasShot()) {
                                stage("no shot from the launch spot");
                                return waitMs(500);
                            }
                            int shot = held;
                            held = 0;
                            return sequential(robot.startShooting(), waitMs(RobotConstants.Gate.AUTO_SHOOT_MS),
                                    robot.stopShooting(), lazy(() -> scored(cell, shot)));
                        }))));
    }

    /** n pollen went into the cell: count them, and take the hive as tipped once that's enough to tip it. */
    private Command scored(Hive cell, int n) {
        if (MatchState.hive != cell) { // hive vision saw it tip already
            load = 0;
            return instant(() -> {});
        }
        load += n;
        if (load < TIP_LOAD) return instant(() -> {});
        load = 0;
        tips++;
        // As the operator would: hive vision then gives it a moment before it overrules this
        return robot.setHive(cell.flipped());
    }

    private double timeLeft() {
        return AUTO_S - clock.seconds();
    }

    /** Time the drive to the park spot needs from here. */
    private double parkTime() {
        Pose at = follower.pose();
        return Math.hypot(park.x() - at.x(), park.y() - at.y()) / PARK_SPEED + PARK_MARGIN_S;
    }

    private boolean timeToPark() {
        return timeLeft() < parkTime();
    }

    private void stage(String stage) {
        this.stage = stage;
        if (MatchLog.active()) {
            Logger.recordOutput("SoloCarry/stage", stage);
            Logger.recordOutput("SoloCarry/held", held);
            Logger.recordOutput("SoloCarry/load", load);
            Logger.recordOutput("SoloCarry/tips", tips);
        }
    }
}
