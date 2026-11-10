package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntFunction;

/**
 * Camera Cycles: as many tips as 30 s allows, from every pollen source on our side of the field, then park.
 *
 * Starts on the audience wall (poses.start), like the other routines. The preloads go into the up cell first (it starts
 * with 3 nectar, so they tip it). Then, cycle after cycle, it takes the source that gives the most pollen for the time
 * (there, take it, on to the up cell's launch spot) and shoots it into whichever cell is up, until no source leaves time
 * to shoot and park; then, if there's time to take one more load before parking, it parks with that for TeleOp. The
 * sources on our half (G402):
 * - the rear-wall and alliance-wall flowers: nose into the bottom with the intake on (G418.B), 4 pollen each;
 * - our garden: the ball camera's pollen chase from where it sees the garden's row;
 * - loose pollen the camera sees close by, such as what spills out of the cell that goes down at a tip.
 *
 * Which cell is up: by count (8 pollen's worth tips a cell; the start cell's 3 nectar are worth 5), with the partner's
 * 4 preloads counted into the cell that comes up at the first tip when it's set to shoot them (init: gamepad1
 * triangle). Hive vision corrects the count whenever it reads the tags. Arriving at a launch spot and seeing none of
 * our tags there (having seen them earlier) means that cell is down, so it goes to the other.
 */
public class CameraCycles extends AutoRoutine {
    // The partner shoots its 4 preloads into the cell that comes up at our first tip (doc/auto-pathing.md). Static so
    // the init pick is kept between runs
    private static boolean partnerShoots = true;

    private static final double AUTO_S = 29; // a start pressed a little late
    // Pollen's worth that tips a cell; the start cell's 3 nectar; the partner's preloads
    private static final double TIP_LOAD = 8, START_LOAD = 5, PARTNER_LOAD = 4;
    // Drive time estimates: at about DRIVE_SPEED, plus DRIVE_MARGIN_S for turning and settling. Every drive gives up
    // after twice its estimate
    private static final double DRIVE_SPEED = 30, DRIVE_MARGIN_S = 0.5;
    // At the launch spot: up to READY_WAIT_S for the launcher (READY_TYPICAL_S planned: it spins up and aims on the
    // way), then the gate open AUTO_SHOOT_MS. Tags seen within TAGS_FRESH_S of arriving count as there; if none show,
    // it looks LOOK_S more
    private static final double READY_WAIT_S = 1.5, READY_TYPICAL_S = 0.3, TAGS_FRESH_S = 0.5, LOOK_S = 0.5;
    private static final double PARK_MARGIN_S = 1;
    // Flowers: the stand-off FLOWER_OUT inches back from the intake spot, pushing in for up to FLOWER_PUSH_S, then the
    // intake on there FLOWER_S
    private static final double FLOWER_OUT = 10, FLOWER_PUSH_S = 1, FLOWER_S = 0.8;
    // Parking: at the rear end of the loading zone (poses.topPark; the partner parks at the audience end), driven to
    // from PARK_LANE inches out from the alliance wall so the drive there doesn't run along the wall into the partner
    private static final double PARK_LANE = 34;
    // The garden chase: at most GARDEN_S, done once nothing more shows for GARDEN_MORE_S after a pickup
    private static final double GARDEN_S = 5, GARDEN_MORE_S = 1;
    // Loose pollen: chased when the camera sees some within LOOSE_RADIUS, for at most LOOSE_S
    private static final double LOOSE_RADIUS = 40, LOOSE_S = 4;

    private final ElapsedTime clock = new ElapsedTime();
    private final int ourHive;
    private final Pose park;
    private final List<Source> sources = new ArrayList<>();

    private int held = RobotConstants.Intake.CAPACITY; // the preload
    private Hive up = Hive.RIGHT;                      // the up cell, as this routine last knew it
    private double load = START_LOAD;                  // pollen's worth in it
    private int tips = 0;
    private String stage = "start";

    /** Somewhere to get pollen: where it goes, about how long taking it takes, and how many it should get. */
    private static final class Source {
        final String name;
        final Pose at;
        final double takeS;
        final int balls;
        final IntFunction<Command> take; // given the room left; adds what it got to held
        boolean used;

        Source(String name, Pose at, double takeS, int balls, IntFunction<Command> take) {
            this.name = name;
            this.at = at;
            this.takeS = takeS;
            this.balls = balls;
            this.take = take;
        }
    }

    public CameraCycles(Robot robot, Alliance alliance) {
        super(robot, alliance);
        ourHive = alliance == Alliance.RED ? HiveTags.RED : HiveTags.BLUE;
        park = poses.topPark;

        sources.add(flower("rear flower", poses.flowerIntake));
        sources.add(flower("wall flower", poses.wallFlowerIntake));
        sources.add(new Source("garden", poses.gardenLook, 3, 4, this::garden));
    }

    @Override
    public Pose startPose() {
        return poses.start;
    }

    @Override
    public boolean usesBallVision() {
        return true;
    }

    @Override
    public void initLoop(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.triangleWasPressed()) partnerShoots = !partnerShoots;
        telemetry.addData("Partner", "%s   (triangle)",
                partnerShoots ? "shoots its preloads after our first tip" : "doesn't shoot");
    }

    @Override
    public void setUp() {
        robot.hiveVision.setEnabled(true);
        robot.ballVision.keepToHalf(alliance); // G402: our side of the field in AUTO
        MatchState.hive = Hive.RIGHT;
        clock.reset(); // the match starts now
    }

    @Override
    public Command autoRoutine() {
        Command carry = sequential(lazy(this::shootLoad), lazy(this::cycle));
        return sequential(
                deadline(carry.raceWith(waitUntil(this::timeToPark)), robot.aimAtHive(), robot.enableFlywheel()),
                robot.stopShooting(),
                robot.intake.off(),
                robot.launcher.idle(),
                instant(() -> stage("parking")),
                lazy(() -> {
                    List<Command> legs = new ArrayList<>();
                    for (Pose p : parkRoute(follower.pose())) legs.add(drive(p));
                    return sequential(legs.toArray(new Command[0]));
                }),
                instant(() -> stage("parked")));
    }

    // ---- Cycles ----

    /**
     * The best source that still leaves time to shoot and park, taken and shot; then the next. None: a load to park
     * with for TeleOp, if there's time to take one.
     */
    private Command cycle() {
        syncHive();
        Source next = best(true);
        if (next != null) {
            next.used = true;
            stage("taking " + next.name);
            return sequential(next.take.apply(RobotConstants.Intake.CAPACITY - held), lazy(this::shootLoad),
                    lazy(this::cycle));
        }
        Source keep = held < RobotConstants.Intake.CAPACITY ? best(false) : null;
        if (keep == null) {
            stage("no time for another cycle");
            return instant(() -> {});
        }
        keep.used = true;
        stage("taking " + keep.name + " for TeleOp");
        return sequential(keep.take.apply(RobotConstants.Intake.CAPACITY - held),
                instant(() -> stage("holding " + held + " for TeleOp")));
    }

    /**
     * Most pollen per second, of the sources that leave time to shoot (toShoot; otherwise just to take it) and park;
     * null if none does.
     */
    private Source best(boolean toShoot) {
        List<Source> options = new ArrayList<>();
        for (Source s : sources) if (!s.used) options.add(s);
        Source loose = loose();
        if (loose != null) options.add(loose);

        int room = RobotConstants.Intake.CAPACITY - held;
        Pose here = follower.pose(), launch = poses.launch(up);
        Source best = null;
        double bestRate = 0;
        for (Source s : options) {
            double cycle = driveTime(here, s.at) + s.takeS + (toShoot ? driveTime(s.at, launch) : 0);
            double after = toShoot ? shootTime() + parkTime(launch) : parkTime(s.at);
            if (cycle + after > timeLeft()) continue;
            double rate = Math.min(s.balls, room) / cycle;
            if (rate > bestRate) {
                best = s;
                bestRate = rate;
            }
        }
        return best;
    }

    /** Pollen the camera sees within LOOSE_RADIUS (on our half), as a source where the robot is; null if none. */
    private Source loose() {
        Pose here = follower.pose();
        int n = 0;
        for (double[] p : robot.ballVision.tracker().chaseable()) {
            if (Math.hypot(p[0] - here.x(), p[1] - here.y()) < LOOSE_RADIUS) n++;
        }
        if (n == 0) return null;
        return new Source("loose pollen", here, LOOSE_S, n, room -> chase(room, LOOSE_S, GARDEN_MORE_S));
    }

    private Source flower(String name, Pose intakeAt) {
        double h = intakeAt.heading();
        Pose out = new Pose(intakeAt.x() - FLOWER_OUT * Math.cos(h), intakeAt.y() - FLOWER_OUT * Math.sin(h), h);
        return new Source(name, out, FLOWER_PUSH_S + FLOWER_S, 4, room -> sequential(
                drive(out),
                instant(() -> stage("into the " + name)),
                robot.intake.intake(),
                robot.driveTo(intakeAt).raceWith(waitMs((long) (FLOWER_PUSH_S * 1000))),
                waitMs((long) (FLOWER_S * 1000)),
                instant(() -> held = Math.min(RobotConstants.Intake.CAPACITY, held + 4))));
    }

    /** From where the camera sees the garden's row, the pollen chase. */
    private Command garden(int room) {
        return sequential(drive(poses.gardenLook), chase(room, GARDEN_S, GARDEN_MORE_S));
    }

    /** The pollen chase, up to room balls; adds what it counted (it counts high: shooting empties the robot anyway). */
    private Command chase(int room, double timeoutS, double moreS) {
        PollenChase chase = new PollenChase(robot, room, timeoutS).endingAfter(moreS);
        return sequential(chase.command(), lazy(() -> {
            held = Math.min(RobotConstants.Intake.CAPACITY, held + chase.getPickedUp());
            return instant(() -> {});
        }));
    }

    // ---- Shooting ----

    /**
     * Everything held into the up cell, from its launch spot, if there's time to and still park. Its tags not in view
     * there (and seen before) means that cell is down: then to the other cell's spot.
     */
    private Command shootLoad() {
        syncHive();
        if (held == 0) return instant(() -> {});
        Hive cell = up;
        if (driveTime(follower.pose(), poses.launch(cell)) + shootTime() + parkTime(poses.launch(cell)) > timeLeft()) {
            stage("no time to shoot " + held);
            return instant(() -> {});
        }
        stage("to the launch spot for " + cell);
        boolean[] wasReady = {false};
        double[] arrived = {0};
        return sequential(
                drive(poses.launch(cell)),
                instant(() -> arrived[0] = FusedLocalizer.now()),
                waitUntil(() -> {
                    wasReady[0] |= robot.launcher.isReady();
                    boolean looked = tagsSince(arrived[0] - TAGS_FRESH_S) || !tagsEverSeen()
                            || FusedLocalizer.now() - arrived[0] >= LOOK_S;
                    return wasReady[0] && looked;
                }).raceWith(waitMs((long) (READY_WAIT_S * 1000))),
                lazy(() -> {
                    syncHive();
                    if (up != cell) return shootLoad(); // hive vision says the other cell's up
                    if (tagsEverSeen() && !tagsSince(arrived[0] - TAGS_FRESH_S)) {
                        // Down: tipped by more than the count knew (the partner's shots, say)
                        stage(cell + " is down");
                        cameUp(cell.flipped(), TIP_LOAD / 2);
                        return sequential(robot.setHive(up), lazy(this::shootLoad));
                    }
                    if (!wasReady[0] && !robot.launcher.hasShot()) {
                        stage("no shot from the launch spot: " + robot.launcher.getStatus());
                        return waitMs(500);
                    }
                    stage("shooting " + held + " into " + cell);
                    int shot = held;
                    held = 0;
                    return sequential(robot.startShooting(), waitMs(RobotConstants.Gate.AUTO_SHOOT_MS),
                            robot.stopShooting(), lazy(() -> scored(shot)));
                }));
    }

    /** n pollen went into the up cell: a tip once the count reaches TIP_LOAD. */
    private Command scored(int n) {
        load += n;
        if (load < TIP_LOAD) return instant(() -> {});
        tips++;
        stage("tipped " + up);
        cameUp(up.flipped(), tips == 1 && partnerShoots ? PARTNER_LOAD : 0);
        return robot.setHive(up);
    }

    /** The other cell came up, with this much in it as far as anyone knows. */
    private void cameUp(Hive cell, double inIt) {
        up = cell;
        load = inIt;
    }

    /** Hive vision set the other cell up: it tipped without the count knowing. */
    private void syncHive() {
        if (MatchState.hive == up) return;
        tips++;
        cameUp(MatchState.hive, tips == 1 && partnerShoots ? PARTNER_LOAD : 0);
    }

    private boolean tagsEverSeen() {
        return filter().lastSeen(ourHive) > Double.NEGATIVE_INFINITY;
    }

    private boolean tagsSince(double time) {
        return filter().lastSeen(ourHive) > time;
    }

    private PoseFilter filter() {
        return robot.drivebase.getLocalizer().getFilter();
    }

    // ---- Time ----

    /** Drives there, giving up after twice the time it should take (a ball or a robot in the way). */
    private Command drive(Pose to) {
        return lazy(() -> robot.driveTo(to).raceWith(waitMs((long) (2000 * driveTime(follower.pose(), to)))));
    }

    private static double driveTime(Pose from, Pose to) {
        return Math.hypot(to.x() - from.x(), to.y() - from.y()) / DRIVE_SPEED + DRIVE_MARGIN_S;
    }

    private static double shootTime() {
        return READY_TYPICAL_S + RobotConstants.Gate.AUTO_SHOOT_MS / 1000.0;
    }

    private double parkTime(Pose from) {
        double t = PARK_MARGIN_S;
        for (Pose p : parkRoute(from)) {
            t += driveTime(from, p);
            from = p;
        }
        return t;
    }

    /**
     * Out to PARK_LANE (from the wall side, short of the zone: along the wall the partner's in the way), across to level
     * with the park spot, and in.
     */
    private List<Pose> parkRoute(Pose from) {
        Pose red = alliance.apply(from), parkRed = alliance.apply(park);
        List<Pose> route = new ArrayList<>();
        if (red.x() < PARK_LANE && red.y() < parkRed.y() - 8) {
            route.add(poseFactory.of(PARK_LANE, Math.max(red.y(), 60), 0));
        }
        route.add(poseFactory.of(PARK_LANE, parkRed.y(), 0));
        route.add(park);
        return route;
    }

    private double timeLeft() {
        return AUTO_S - clock.seconds();
    }

    private boolean timeToPark() {
        return timeLeft() < parkTime(follower.pose());
    }

    @Override
    public String status() {
        return String.format("%s  (%.1f s, held %d, %s load %.0f, tips %d)", stage, clock.seconds(), held, up, load, tips);
    }

    private void stage(String stage) {
        this.stage = stage;
        if (MatchLog.active()) {
            Logger.recordOutput("CameraCycles/stage", stage);
            Logger.recordOutput("CameraCycles/held", held);
            Logger.recordOutput("CameraCycles/load", load);
            Logger.recordOutput("CameraCycles/tips", tips);
        }
    }
}
