package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.RobotConstants.Vision.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Hive;
import org.firstinspires.ftc.teamcode.MatchState;
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
    private boolean running = false;

    private List<Seen> lastSeen = Collections.emptyList();
    private HiveReader.Frame lastFrame = null;
    private long lastStaleness = -1;
    private int switches = 0;

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

    public boolean isAvailable() {
        return limelight != null;
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
        if (enabled && limelight != null) {
            limelight.start();
            running = true;
        }
    }

    @Override
    public void update() {
        if (!running) return;
        LLResult result = limelight.getLatestResult();
        if (result == null) return;
        lastStaleness = result.getStaleness();
        if (!result.isValid() || lastStaleness > MAX_STALENESS_MS) return;

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

        Hive next = tracker.onFrame(now(), result.getControlHubTimeStampNanos(), lastFrame.up, lastFrame.conflict,
                MatchState.hive);
        if (next != null) {
            MatchState.hive = next;
            switches++;
        }
    }

    @Override
    public void stop() {
        if (running) limelight.stop();
        running = false;
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
        if (tracker.isLocked()) return "AUTO HIVE OFF";
        return "auto";
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Hive vision", status());
        if (!running) return;
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
