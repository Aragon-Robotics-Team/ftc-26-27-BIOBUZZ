package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.BallVision.*;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Drive.SLOW_MODE_SCALE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.VideoRecorder;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

/**
 * Drive around and record the ball camera, to make a dataset from later. Recording starts at Play; cross stops it and
 * starts a new clip, so each stretch of driving can be its own file. Clips go to /sdcard/FIRST/data/recordings: download
 * them from http://192.168.43.1:8080/recordings on the robot's Wi-Fi (see {@link VideoRecorder} for the files).
 *
 * Gamepad 1 drives (field-centric from where the robot faces at start, like the Ball Vision Test): left stick drives,
 * right stick x turns, hold the left trigger for slow mode.
 *
 * Exposure and gain start at RobotConstants.BallVision's, so the frames look like what the robot will see; change them
 * under Configurables > RecordVideo in Panels (they apply right away). Frames are the full 1280x800 capture, about
 * 150-250 KB each: at 15 fps that's 2-4 MB a second, so delete old clips off the robot once they're copied.
 */
@Configurable
@TeleOp(name = "Record Video", group = "Tuning")
public class RecordVideo extends OpMode {
    public static double exposureMs = EXPOSURE_MS;
    public static int gain = GAIN;
    public static double maxFps = 15;       // 0 keeps every frame (about 30 a second, if the writer keeps up)
    public static int jpegQuality = 90;
    public static boolean preview = true;   // Driver Station camera stream; costs some CPU

    private Drivetrain drivetrain;
    private VideoRecorder recorder;
    private VisionPortal portal;
    private TelemetryManager panels;
    private double appliedExposure = Double.NaN;
    private int appliedGain = -1;

    @Override
    public void init() {
        Scheduler.reset();
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setPose(new Pose(0, 0, 0));
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        recorder = new VideoRecorder(jpegQuality, maxFps);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new android.util.Size(CAPTURE_WIDTH, CAPTURE_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(recorder)
                .enableLiveView(preview)
                .build();
    }

    @Override
    public void init_loop() {
        applyCameraSettings();
        telemetry.addData("Camera", portal.getCameraState());
        telemetry.addLine("Press Play to start recording");
        telemetry.update();
    }

    @Override
    public void start() {
        drivetrain.start();
        schedule(drivetrain.driveFieldCentric(
                () -> -gamepad1.left_stick_y * driveScale(),
                () -> -gamepad1.left_stick_x * driveScale(),
                () -> -gamepad1.right_stick_x * driveScale(),
                0));
        recorder.start();
    }

    private double driveScale() {
        return gamepad1.left_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
    }

    @Override
    public void loop() {
        if (gamepad1.crossWasPressed()) {
            if (recorder.isRecording()) recorder.stop();
            else recorder.start();
        }
        drivetrain.update();
        Scheduler.execute();
        Pose pose = drivetrain.getPose();
        recorder.setPose(pose.x(), pose.y(), pose.heading());
        applyCameraSettings();

        panels.addData("Recording", recorder.status());
        panels.addData("Camera", String.format("%s, %.0f fps", portal.getCameraState(), portal.getFps()));
        panels.addData("Exposure", String.format("%.1f ms, gain %d", exposureMs, gain));
        panels.addLine(recorder.isRecording() ? "cross: stop this clip" : "cross: start a new clip");
        panels.update(telemetry);
    }

    /** Manual exposure and gain, again whenever they change (0 ms is auto exposure), like BallVision. */
    private void applyCameraSettings() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        if (exposureMs == appliedExposure && gain == appliedGain) return;
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);
        if (exposure == null) return;
        if (exposureMs > 0) {
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure((long) (exposureMs * 1000), TimeUnit.MICROSECONDS);
            if (gainControl != null) {
                gainControl.setGain(Math.max(gainControl.getMinGain(), Math.min(gainControl.getMaxGain(), gain)));
            }
        } else {
            exposure.setMode(ExposureControl.Mode.Auto);
        }
        appliedExposure = exposureMs;
        appliedGain = gain;
    }

    @Override
    public void stop() {
        recorder.stop();
        Scheduler.reset();
        drivetrain.stop();
        portal.close();
    }
}
