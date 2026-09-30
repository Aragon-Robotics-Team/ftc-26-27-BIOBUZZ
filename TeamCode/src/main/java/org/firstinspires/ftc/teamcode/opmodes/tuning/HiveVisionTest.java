package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.HiveVision;
import org.firstinspires.ftc.teamcode.vision.HiveReader;

/**
 * Check and calibrate the Limelight's hive reading on the practice hive. Edit the values under
 * Configurables > HiveVisionTest in Panels; copy the final ones into RobotConstants.Vision.
 *
 * Pick the alliance during init (X blue, B red). Every tag in view is listed with its ID, which of our cells it's on,
 * its camera-space position, its height above the tiles and how far off its face the camera is looking.
 * 1. Lens height: measure it and set lensHeight.
 * 2. Tilt: measure the height of one tag on the practice hive (tape to the tag's center) and set knownTagId and
 *    knownTagHeight. With that tag in view, "Tilt from known tag" is the camera tilt; set tiltDegrees to it.
 * 3. Threshold: read the heights with each cell up; set threshold halfway between (about 42 in).
 * 4. Walk the robot around the hive to find the real range and the largest angle off a tag's face that still reads.
 */
@Configurable
@TeleOp(name = "Hive Vision Test", group = "Tuning")
public class HiveVisionTest extends OpMode {
    public static double lensHeight = RobotConstants.Vision.LENS_HEIGHT;
    public static double tiltDegrees = Math.toDegrees(RobotConstants.Vision.TILT);
    public static double threshold = RobotConstants.Vision.HEIGHT_THRESHOLD;
    public static int knownTagId = 34;
    public static double knownTagHeight = 49.7;

    private HiveVision vision;
    private TelemetryManager panels;

    @Override
    public void init() {
        vision = new HiveVision(hardwareMap);
        vision.setEnabled(true);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        MatchState.selectAlliance(gamepad1, telemetry);
        telemetry.addData("Limelight", vision.isAvailable() ? "found" : "NOT FOUND (check the configuration name)");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        // HiveVision reads these every loop, so edits in Panels take effect right away
        RobotConstants.Vision.LENS_HEIGHT = lensHeight;
        RobotConstants.Vision.TILT = Math.toRadians(tiltDegrees);
        RobotConstants.Vision.HEIGHT_THRESHOLD = threshold;

        vision.update();

        HiveReader.Frame frame = vision.getLastFrame();
        panels.addData("Alliance", MatchState.alliance);
        panels.addData("Vision", vision.status());
        panels.addData("Frame says", frame == null || frame.readings.isEmpty() ? "no tags of ours"
                : frame.conflict ? "our tags disagree" : frame.up + " cell up");
        panels.addData("MatchState.hive", MatchState.hive);

        double tilt = Double.NaN;
        for (HiveVision.Seen s : vision.getLastSeen()) {
            panels.addLine(String.format("Tag %d %s: x %.1f y %.1f z %.1f in | height %.1f in | %.0f° off its face",
                    s.id, s.cell == null ? "(not ours)" : "on " + s.cell, s.x, s.y, s.z, s.height, s.offFaceDegrees));
            if (s.id == knownTagId) tilt = HiveReader.tiltFor(s.y, s.z, lensHeight, knownTagHeight);
        }
        panels.addData("Tilt from known tag", Double.isNaN(tilt)
                ? "tag " + knownTagId + " not in view"
                : String.format("%.1f° (tag %d at %.1f in)", Math.toDegrees(tilt), knownTagId, knownTagHeight));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        vision.stop();
    }
}
