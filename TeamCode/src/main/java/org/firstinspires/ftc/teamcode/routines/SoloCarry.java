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
import org.firstinspires.ftc.teamcode.core.HiveTags;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.PollenChase;
import org.firstinspires.ftc.teamcode.core.PoseFilter;
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.psilynx.psikit.core.Logger;

/**
 * Carries the alliance alone: as many tips as 30 s allows, with a partner that only starts in our LEFT corner (back to
 * the alliance wall, by the loading zone) and parks. Starts back against the alliance wall on tile A2 (allianceWallStart).
 *
 * The up cell starts with 3 nectar in it (5 pollen's worth), so the 4 preloads tip it. After that it's cycles of the
 * pollen chase on the ball camera from a lookout (the garden first), shooting whatever it got into the cell that's
 * up; with nothing in sight it tries the next lookout.
 *
 * Which cell is up: the hive's tags face the up cell's side, so at a cell's launch spot (on its side) the Limelight
 * sees them while that cell is up and loses them once it tips. Arriving to shoot and after shooting, it goes by that
 * once it has seen tags at all this match; with none ever seen (Limelight down), by a count of what it has put into
 * the up cell: 8 pollen tip a hive (Field Setup Guide 12.3). Hive vision also sets the up cell whenever it can tell.
 *
 * Time: each chase is given what's left after driving to the launch spot, shooting and parking from there; a load
 * it can't shoot and still park in time stays in the robot. It parks whatever it's doing when the drive there would
 * take all that's left, by the audience side of the loading zone (clear of the partner) and in along the wall to the
 * zone's audience end, so the partner should park at the rear end. It counts 29 s, for a start pressed late. Every
 * drive gives up after twice the time it should take. It can't take pollen from the flowers.
 */
public class SoloCarry extends AutoRoutine {
    private static final double AUTO_S = 29;
    // Pollen's worth that tips a hive, and what the up cell starts with: 3 nectar, each 5/3 of a pollen (the sim's
    // weights, from 8 pollen tipping a hive and 3 pollen with 3 nectar tipping it)
    private static final double TIP_LOAD = 8, START_LOAD = 5;
    // Drive time estimates, here to there: at about DRIVE_SPEED (the sim's robot manages 40 in/s), plus DRIVE_MARGIN_S
    // for turning and settling
    private static final double DRIVE_SPEED = 30, DRIVE_MARGIN_S = 0.5;
    // At the launch spot: up to READY_WAIT_S for the launcher (READY_TYPICAL_S planned for: it spins up and aims on
    // the way), AUTO_SHOOT_MS with the gate open, up to TIP_CHECK_S after for the tags to show a tip. Parking is
    // planned with PARK_MARGIN_S to spare, and starts when that's all that's left
    private static final double READY_WAIT_S = 1.5, READY_TYPICAL_S = 0.5, TIP_CHECK_S = 0.8, PARK_MARGIN_S = 1;
    // The shortest chase worth starting; tags seen within TAGS_FRESH_S count as in view now
    private static final double MIN_CHASE_S = 2, TAGS_FRESH_S = 0.4;
    // A chase with something in hand that's found no more for this long goes to shoot it
    private static final double MORE_S = 4;

    private final ElapsedTime clock = new ElapsedTime();
    private final Pose[] lookouts;
    // The audience end of the loading zone, facing along the wall, the front 2 in into the zone, which leaves the
    // partner the rear 22 in of it (park it there). Reached from approach, out from the zone's audience end, so the
    // drive in doesn't cut across the partner
    private final Pose park, approach;
    private final int ourHive;
    private int nextLookout = 0;
    private int held = RobotConstants.Intake.CAPACITY; // the preload
    private double load = START_LOAD;                 // pollen's worth in the up cell, as far as it knows
    private int tips = 0;
    private String stage = "start";

    public SoloCarry(Robot robot, Alliance alliance) {
        super(robot, alliance);
        lookouts = new Pose[] {poses.gardenLook, poses.audienceSpillLook, poses.rearSpillLook};
        park = poseFactory.of(9, 96 + 2 - RobotConstants.Field.ROBOT_LENGTH / 2, 90);
        approach = poseFactory.of(20, 80, 90);
        ourHive = alliance == Alliance.RED ? HiveTags.RED : HiveTags.BLUE;
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
                lazy(() -> sequential(drive(lookouts[nextLookout++]), lazy(this::cycle))));
        return sequential(
                deadline(carry.raceWith(waitUntil(this::timeToPark)), robot.aimAtHive(), robot.enableFlywheel()),
                robot.stopShooting(),
                robot.intake.off(),
                instant(() -> stage("parking")),
                drive(approach),
                drive(park),
                instant(() -> stage("parked")));
    }

    /**
     * Chase from here, leaving time to shoot what it gets and park; with nothing found, the next lookout and chase
     * again. Anything picked up gets shot (a chase only ends short of full when it can't find more). No time for a
     * chase: shoot what it has if that still leaves time to park, and stop.
     */
    private Command cycle() {
        double spare = timeLeft() - shootAndParkTime(follower.pose());
        if (held == RobotConstants.Intake.CAPACITY || (held > 0 && spare < MIN_CHASE_S)) {
            if (spare < 0) return instant(() -> stage("no time to shoot " + held));
            return sequential(lazy(this::driveAndShoot), lazy(this::cycle));
        }
        if (spare < MIN_CHASE_S) return instant(() -> stage("no time for more"));
        int room = RobotConstants.Intake.CAPACITY - held;
        PollenChase chase = new PollenChase(robot, room, spare).endingAfter(MORE_S);
        stage("chasing for " + room);
        return sequential(
                chase.command(),
                robot.intake.off(),
                lazy(() -> {
                    // The chase counts high rather than low (it can't see inside); shooting empties the robot anyway
                    held = Math.min(RobotConstants.Intake.CAPACITY, held + chase.getPickedUp());
                    if (held > 0) return sequential(lazy(this::driveAndShoot), lazy(this::cycle)); // full, or no more found
                    Pose lookout = lookouts[nextLookout++ % lookouts.length];
                    stage("nothing in sight: to a lookout");
                    return sequential(drive(lookout), lazy(this::cycle));
                }));
    }

    /**
     * To the launch spot for the up cell; if the tags say that cell's down, to the other one's. Then everything into
     * it, if the launcher has a shot with the turret on it, and a check for a tip.
     */
    private Command driveAndShoot() {
        Hive cell = MatchState.hive;
        stage("to the launch spot for " + cell);
        boolean[] wasReady = {false};
        return sequential(
                drive(poses.launch(cell)),
                waitUntil(() -> wasReady[0] = robot.launcher.isReady()).raceWith(waitMs((long) (READY_WAIT_S * 1000))),
                lazy(() -> {
                    Hive up = upFromTags(cell, READY_WAIT_S);
                    if (up != null && up != cell) {
                        stage(cell + " is down by the tags");
                        return sequential(robot.setHive(up), lazy(this::driveAndShoot));
                    }
                    if (MatchState.hive != cell) return driveAndShoot(); // hive vision changed it on the way
                    // Ready at some point in the wait (it wobbles as the robot settles), or a shot with the turret on it
                    if (!wasReady[0] && !(robot.launcher.hasShot() && robot.launcher.isTurretOnTarget())) {
                        stage("no shot from the launch spot: " + robot.launcher.getStatus());
                        return waitMs(500);
                    }
                    stage("shooting " + held + " into " + cell);
                    int shot = held;
                    held = 0;
                    double shotAt = FusedLocalizer.now();
                    return sequential(robot.startShooting(), waitMs(RobotConstants.Gate.AUTO_SHOOT_MS),
                            robot.stopShooting(), lazy(() -> scored(cell, shot, shotAt)));
                }));
    }

    /**
     * n pollen went into the cell, shot from shotAt. Did it tip? With the tags in view when it shot: whether they're
     * gone or the arm went over, within TIP_CHECK_S (only waited for when the count says it should have tipped).
     * Without them, by the count.
     */
    private Command scored(Hive cell, int n, double shotAt) {
        double expected = load + n;
        if (!tagsSince(shotAt - READY_WAIT_S - 1)) return countScored(cell, expected);
        return sequential(
                waitUntil(() -> tippedByTags(cell)).raceWith(waitMs(expected >= TIP_LOAD ? (long) (TIP_CHECK_S * 1000) : 0)),
                lazy(() -> {
                    if (tippedByTags(cell)) return tipped(cell);
                    load = Math.min(expected, TIP_LOAD - 1); // still up: whatever the count says, it's short of a tip
                    return instant(() -> {});
                }));
    }

    /** By the count alone: n more, and a tip once that's enough to tip it. */
    private Command countScored(Hive cell, double expected) {
        if (MatchState.hive != cell) return tipped(cell); // hive vision saw it tip already
        load = expected;
        return load < TIP_LOAD ? instant(() -> {}) : tipped(cell);
    }

    private Command tipped(Hive cell) {
        load = 0;
        tips++;
        stage("tipped " + cell);
        // As the operator would: hive vision then gives it a moment before it overrules this
        return MatchState.hive == cell ? robot.setHive(cell.flipped()) : instant(() -> {});
    }

    /** At cell's launch spot: hive vision or the arm says the other cell's up, or our tags have gone from view. */
    private boolean tippedByTags(Hive cell) {
        return MatchState.hive != cell || armUp() != cell || !tagsSince(FusedLocalizer.now() - TAGS_FRESH_S);
    }

    /**
     * Which cell the tags say is up, at cell's launch spot, after looking for within seconds: by the arm if ours are
     * in view, the other cell if they've been seen this match but aren't now; null if never (the Limelight's down).
     */
    private Hive upFromTags(Hive cell, double within) {
        if (filter().lastSeen(ourHive) == Double.NEGATIVE_INFINITY) return null;
        if (tagsSince(FusedLocalizer.now() - within)) return armUp();
        return cell.flipped();
    }

    private Hive armUp() {
        return HiveTags.up(alliance, filter().arm(ourHive));
    }

    private boolean tagsSince(double time) {
        return filter().lastSeen(ourHive) > time;
    }

    private PoseFilter filter() {
        return robot.drivebase.getLocalizer().getFilter();
    }

    /** Drives there, giving up after twice the time it should take (a ball or the partner in the way). */
    private Command drive(Pose to) {
        return lazy(() -> robot.driveTo(to).raceWith(waitMs((long) (2000 * driveTime(follower.pose(), to)))));
    }

    private static double driveTime(Pose from, Pose to) {
        return Math.hypot(to.x() - from.x(), to.y() - from.y()) / DRIVE_SPEED + DRIVE_MARGIN_S;
    }

    /** From there: to the up cell's launch spot, shooting there, and parking from there. */
    private double shootAndParkTime(Pose from) {
        Pose launch = poses.launch(MatchState.hive);
        return driveTime(from, launch) + READY_TYPICAL_S + RobotConstants.Gate.AUTO_SHOOT_MS / 1000.0
                + TIP_CHECK_S / 2 + parkTime(launch);
    }

    private double timeLeft() {
        return AUTO_S - clock.seconds();
    }

    /** Time the drive to the park spot needs from there, by the approach. */
    private double parkTime(Pose from) {
        return driveTime(from, approach) + driveTime(approach, park) + PARK_MARGIN_S;
    }

    private boolean timeToPark() {
        return timeLeft() < parkTime(follower.pose());
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
