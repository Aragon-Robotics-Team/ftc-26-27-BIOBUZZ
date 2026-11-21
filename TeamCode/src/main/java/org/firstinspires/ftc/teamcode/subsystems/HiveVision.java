package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Vision.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.core.AimCorrection;
import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.HiveTags;
import org.firstinspires.ftc.teamcode.core.MatchState;
import org.firstinspires.ftc.teamcode.core.PoseFilter;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.FusedLocalizer;
import org.firstinspires.ftc.teamcode.vision.HiveReader;
import org.firstinspires.ftc.teamcode.vision.HiveTracker;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Keeps MatchState.hive right from the hive's AprilTags, seen by the Limelight 3A on the turret.
 * HiveReader turns a frame into a hive state (tag ID = which cell, tag height = up or down) and HiveTracker decides
 * when to switch. Tags only face the side of the field where the up cell is, so this confirms and corrects the state
 * once the robot is on the shooting side; it doesn't catch a tip from behind.
 *
 * Off unless enabled (TeleOp enables it; Auto keeps its own flips). If the Limelight isn't in the configuration,
 * everything else still runs and telemetry says so.
 *
 * Also hands every hive tag it sees to the drivetrain's FusedLocalizer, to correct the pose (fuseInto, when
 * Localization.FUSE_HIVE_TAGS): then the camera runs whether or not the hive state is enabled. Each frame goes in once,
 * with the turret angle from when it was taken (frames are a few tens of ms old by the time they're read).
 *
 * And, while Vision.AIM_FROM_TAGS, moves the aim target by how far the up cell's tags are from where the pose puts them
 * (core.AimCorrection, aimTarget): the shot follows the camera, the pose stays the Pinpoint's.
 */
public class HiveVision implements Subsystem {
    /** Every tag in the last frame, ours or not, for the Hive Vision Test OpMode. */
    public static class Seen {
        public final int id;
        public final double x, y, z;       // camera space, inches
        public final double height;         // inches above the tiles
        public final double offFaceDegrees; // angle between the camera and the tag's face normal
        public final Hive cell;             // our cell it's on, or null

        Seen(int id, double x, double y, double z, double height, double offFaceDegrees, Hive cell) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
            this.height = height;
            this.offFaceDegrees = offFaceDegrees;
            this.cell = cell;
        }
    }

    private final Limelight3A limelight; // null if it isn't configured
    private final HiveTracker tracker = new HiveTracker(FRAMES_TO_SWITCH, MANUAL_HOLDOFF_S, MANUAL_DISAGREE_S);
    private boolean enabled = false;
    private boolean running = false, checking = false; // checking: started by tagsInView, before start()
    private double lastSwitchTry = Double.NEGATIVE_INFINITY;
    private int flat; // tagsInView(): hive tags seen without a 3D pose

    private List<Seen> lastSeen = Collections.emptyList();
    private HiveReader.Frame lastFrame = null;
    private long lastStaleness = -1;
    private int switches = 0;

    private FusedLocalizer localizer;   // null: tags don't go to the pose
    // Turret angle each loop (a ring of the last TURRET_HISTORY), to look up where it was when a frame was taken
    private final double[] turretTimes = new double[TURRET_HISTORY], turretAngles = new double[TURRET_HISTORY];
    private int turretCount, turretNext;
    private long lastFused = 0;
    private int fusedTags = 0;
    // The pose each loop, beside the turret angle, for the aim correction
    private final double[] poseX = new double[TURRET_HISTORY], poseY = new double[TURRET_HISTORY],
            poseHeading = new double[TURRET_HISTORY];

    private final AimCorrection aim = new AimCorrection(FusedLocalizer.camera(), AIM_SMOOTHING, MAX_AIM_CORRECTION,
            AIM_HEIGHT_TOLERANCE);
    private long lastAimed = 0;
    private double lastReset = Double.NEGATIVE_INFINITY;

    public HiveVision(HardwareMap hardwareMap) {
        Limelight3A found = null;
        try {
            found = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            found.setPollRateHz(POLL_RATE_HZ);
            found.pipelineSwitch(PIPELINE); // blocking request to the camera: init only
        } catch (RuntimeException e) {
            found = null;
        }
        limelight = found;
    }

    /** Only an enabled HiveVision starts the camera and changes MatchState.hive. Set it before start(). */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /** Also correct this localizer's pose from the tags. Robot gives it the turret angle each loop (record). */
    public void fuseInto(FusedLocalizer localizer) {
        this.localizer = localizer;
    }

    private boolean fusing() {
        return localizer != null && RobotConstants.Localization.FUSE_HIVE_TAGS;
    }

    public boolean isAvailable() {
        return limelight != null;
    }

    /**
     * For a check before the match: how many hive tags the Limelight sees now with a 3D pose, which is all that
     * update() uses (starting it if it isn't yet), or -1 with no Limelight configured; tagsWithout3d() says how many
     * it saw without one. A Limelight still booting when the robot was built missed the pipeline switch:
     * this asks again, once a second, until its results say it's on PIPELINE.
     */
    public int tagsInView() {
        if (limelight == null) return -1;
        if (!checking && !running) {
            limelight.start();
            checking = true;
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.getPipelineIndex() != PIPELINE && now() - lastSwitchTry > 1) {
            lastSwitchTry = now();
            limelight.pipelineSwitch(PIPELINE);
        }
        flat = 0;
        if (result == null || !result.isValid()) return 0;
        int n = 0;
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (HiveTags.hiveOf(f.getFiducialId()) < 0) continue;
            if (f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH).z > 0) n++;
            else flat++; // no 3D pose: the pipeline's Full 3D is off, or its tag size is wrong
        }
        return n;
    }

    /** Hive tags the last tagsInView() saw without a 3D pose, which update() throws away. */
    public int tagsWithout3d() {
        return flat;
    }

    // ---- Operator ----

    /** The operator set the hive by hand; the camera needs longer to overrule that for a moment. */
    public void manualSet() {
        tracker.manualSet(now());
    }

    public boolean isLocked() {
        return tracker.isLocked();
    }

    /** Lock or unlock automatic hive detection. */
    public Command toggleLock() {
        return instant(() -> tracker.setLocked(!tracker.isLocked()));
    }

    // ---- Loop ----

    @Override
    public void start() {
        if ((enabled || fusing() || AIM_FROM_TAGS) && limelight != null) {
            limelight.start();
            running = true;
        }
    }

    @Override
    public void update() {
        // A pose reset makes every aim offset stale (and frames from before it measured the old pose)
        if (localizer != null && localizer.getLastResetTime() != lastReset) {
            lastReset = localizer.getLastResetTime();
            aim.reset();
        }
        if (!running) return;
        LLResult result = limelight.getLatestResult();
        if (result == null) return;
        lastStaleness = result.getStaleness();
        if (!result.isValid() || lastStaleness > MAX_STALENESS_MS) return;
        // The Limelight's own frame time: the hub's timestamp changes on every poll, even of the same frame
        long frameId = Double.doubleToLongBits(result.getTimestamp());
        if (fusing() && frameId != lastFused) {
            lastFused = frameId;
            fuse(result);
        }
        if (AIM_FROM_TAGS && frameId != lastAimed) {
            lastAimed = frameId;
            correctAim(result);
        }
        if (!enabled) return;

        List<HiveReader.Tag> tags = new ArrayList<>();
        List<Seen> seen = new ArrayList<>();
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            if (p.z <= 0) continue; // a pose the camera didn't send comes back as all zeros
            Position c = f.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
            double range = Math.sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
            double off = range > 0 ? Math.toDegrees(Math.acos(Math.min(1, Math.abs(c.z) / range))) : Double.NaN;
            int id = f.getFiducialId();
            tags.add(new HiveReader.Tag(id, p.x, p.y, p.z));
            seen.add(new Seen(id, p.x, p.y, p.z, HiveReader.height(p.y, p.z, LENS_HEIGHT, TILT), off,
                    HiveReader.cellOf(id, MatchState.alliance)));
        }
        lastSeen = seen;
        lastFrame = HiveReader.read(tags, MatchState.alliance, LENS_HEIGHT, TILT, HEIGHT_THRESHOLD);

        Hive next = tracker.onFrame(now(), frameId, lastFrame.up, lastFrame.conflict, MatchState.hive);
        if (next != null) {
            MatchState.hive = next;
            switches++;
        }
    }

    /** Every hive tag in the frame to the localizer, at the moment and turret angle the frame was taken. */
    private void fuse(LLResult result) {
        double taken = now() - (lastStaleness + result.getCaptureLatency() + result.getTargetingLatency()) / 1000;
        double turret = turretAt(taken);
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            if (p.z <= 0) continue;
            if (localizer.addTag(taken, turret, f.getFiducialId(), p.x, p.y, p.z)) fusedTags++;
        }
    }

    /** The up cell's tags in the frame to the aim correction, with the pose and turret angle from when it was taken. */
    private void correctAim(LLResult result) {
        double taken = now() - (lastStaleness + result.getCaptureLatency() + result.getTargetingLatency()) / 1000;
        if (taken < lastReset || turretCount == 0) return;
        List<AimCorrection.Tag> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            if (p.z <= 0) continue;
            tags.add(new AimCorrection.Tag(f.getFiducialId(), p.x, p.y, p.z));
        }
        if (tags.isEmpty()) return;
        aim.addFrame(taken, poseAt(taken), turretAt(taken), tags.toArray(new AimCorrection.Tag[0]),
                MatchState.alliance, MatchState.hive);
    }

    /** Where to aim for a cell: its target moved by what the camera last saw of it (as is with AIM_FROM_TAGS off). */
    public Pose aimTarget(Pose target, Hive cell) {
        return AIM_FROM_TAGS ? aim.apply(target, cell) : target;
    }

    public AimCorrection getAimCorrection() {
        return aim;
    }

    /**
     * The turret's angle and the robot's pose as of time: this loop's readings, stamped with the time of this loop's
     * pose (FusedLocalizer.getLastUpdateTime), so a frame's turret angle and robot pose are from the same moment. Call
     * after the launcher has read the encoder.
     */
    public void record(double time, double turretAngle, Pose pose) {
        if (Double.isNaN(time)) return;
        turretTimes[turretNext] = time;
        turretAngles[turretNext] = turretAngle;
        poseX[turretNext] = pose.x();
        poseY[turretNext] = pose.y();
        poseHeading[turretNext] = pose.heading();
        turretNext = (turretNext + 1) % TURRET_HISTORY;
        turretCount = Math.min(turretCount + 1, TURRET_HISTORY);
    }

    /** The turret's angle at a time, between the two loops either side of it (the oldest or newest kept beyond). */
    private double turretAt(double time) {
        if (turretCount == 0) return 0;
        bracket(time);
        return turretAngles[before] + (turretAngles[after] - turretAngles[before]) * fraction;
    }

    /** The robot's pose at a time, the same way. */
    private Pose poseAt(double time) {
        bracket(time);
        double turn = Angle.normalizeSigned(poseHeading[after] - poseHeading[before]);
        return new Pose(poseX[before] + (poseX[after] - poseX[before]) * fraction,
                poseY[before] + (poseY[after] - poseY[before]) * fraction,
                Angle.normalizeSigned(poseHeading[before] + turn * fraction));
    }

    // bracket()'s answer: the kept loops either side of a time, and how far between them it is
    private int before, after;
    private double fraction;

    private void bracket(double time) {
        double beforeT = Double.NEGATIVE_INFINITY, afterT = Double.POSITIVE_INFINITY;
        int b = -1, a = -1;
        for (int i = 0; i < turretCount; i++) {
            double t = turretTimes[i];
            if (t <= time && t > beforeT) {
                beforeT = t;
                b = i;
            }
            if (t >= time && t < afterT) {
                afterT = t;
                a = i;
            }
        }
        if (b < 0) b = a;
        if (a < 0) a = b;
        before = b;
        after = a;
        fraction = a == b ? 0 : (time - beforeT) / (afterT - beforeT);
    }

    @Override
    public void stop() {
        if (running || checking) limelight.stop();
        running = false;
        checking = false;
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }

    // ---- State ----

    public List<Seen> getLastSeen() {
        return lastSeen;
    }

    /** The last frame's reading of our tags, or null before the first frame. */
    public HiveReader.Frame getLastFrame() {
        return lastFrame;
    }

    /** One word for the drivers: what the camera is doing about the hive. */
    public String status() {
        if (limelight == null) return "no Limelight";
        if (!running) return "off";
        if (!enabled) return "pose only";
        if (tracker.isLocked()) return "AUTO HIVE OFF";
        return "auto";
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Hive vision", status());
        if (AIM_FROM_TAGS && running) {
            Hive up = MatchState.hive;
            if (aim.has(up)) {
                telemetry.addData("Aim from tags", "%s cell %+.1f, %+.1f in (%d tags, %.1f s ago, %d skipped)", up,
                        aim.dx(up), aim.dy(up), aim.tags(up), now() - aim.updated(up), aim.getRejected());
            } else {
                telemetry.addData("Aim from tags", "%s cell not seen yet: aiming from the pose", up);
            }
        }
        if (fusing() && running) {
            PoseFilter filter = localizer.getFilter();
            telemetry.addData("Pose from tags", "%d tags used, %d thrown out, ±%.1f in ±%.1f°", fusedTags,
                    filter.getRejected(), Math.hypot(filter.sigma(PoseFilter.X), filter.sigma(PoseFilter.Y)),
                    Math.toDegrees(filter.sigma(PoseFilter.HEADING)));
            // Resets move the Pinpoint too, so this is only what the tags have done
            Pose odometry = localizer.getOdometryPose();
            telemetry.addData("Tags moved the pose", "%+.1f, %+.1f in, %+.1f° from the Pinpoint's",
                    filter.x() - odometry.x(), filter.y() - odometry.y(),
                    Math.toDegrees(Angle.normalizeSigned(filter.heading() - odometry.heading())));
        }
        if (!running || !enabled) return;
        StringBuilder tags = new StringBuilder();
        for (Seen s : lastSeen) {
            if (s.cell == null) continue;
            if (tags.length() > 0) tags.append(", ");
            tags.append(String.format("%d %.1f in", s.id, s.height));
        }
        telemetry.addData("Hive tags", tags.length() > 0 ? tags : "none");
        String says = lastFrame == null || lastFrame.readings.isEmpty() ? "-"
                : lastFrame.conflict ? "tags disagree" : lastFrame.up + " up";
        telemetry.addData("Hive camera says", "%s (run %d, switches %d, %d ms old)", says, tracker.getRun(),
                switches, lastStaleness);
    }
}
