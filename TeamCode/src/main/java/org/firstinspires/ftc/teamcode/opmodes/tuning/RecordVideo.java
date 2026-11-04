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
import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.vision.VideoRecorder;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

/**
 * Drive around, intake and shoot while recording the ball camera, to make a dataset from later. Recording starts at
 * Play; square stops it and starts a new clip, so each stretch of driving can be its own file. Clips go to /sdcard/FIRST/data/recordings: download
 * them from http://192.168.43.1:8080/recordings on the robot's Wi-Fi (see {@link VideoRecorder} for the files).
 *
 * Gamepad 1 (intake and shooting buttons as in TeleOp):
 *   left stick      field-centric drive (from where the robot faces at start, like the Ball Vision Test)
 *   right stick x   turn                 hold L2  slow mode
 *   triangle        toggle intake        cross    toggle outtake
 *   R1              flywheel on          L1       flywheel off
 *   hold R2         open the gate and feed the shooter
 *   square          stop this clip / start a new one
 * The robot doesn't know where it is on the field here, so nothing aims: the flywheel spins at the fixed
 * RobotConstants.Launcher.TARGET_VELOCITY (as in the Flywheel Tuner) and the turret holds where it is.
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

    private Robot robot;
    private VideoRecorder recorder;
    private VisionPortal portal;
    private TelemetryManager panels;
    private double appliedExposure = Double.NaN;
    private int appliedGain = -1;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(hardwareMap); // ball vision stays off: the recorder has the webcam
        robot.drivebase.setPose(new Pose(0, 0, 0));
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
        robot.start();
        schedule(robot.drivebase.driveFieldCentric(
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
        if (gamepad1.squareWasPressed()) {
            if (recorder.isRecording()) recorder.stop();
            else recorder.start();
        }
        bindControls();
        robot.update();
        Scheduler.execute();
        Pose pose = robot.drivebase.getPose();
        recorder.setPose(pose.x(), pose.y(), pose.heading());
        applyCameraSettings();

        panels.addData("Recording", recorder.status());
        panels.addData("Camera", String.format("%s, %.0f fps", portal.getCameraState(), portal.getFps()));
        panels.addData("Exposure", String.format("%.1f ms, gain %d", exposureMs, gain));
        panels.addData("Intake", robot.intake.getMode() + (robot.intake.isFeeding() ? " (feeding)" : ""));
        panels.addData("Flywheel", String.format("%.0f / %.0f", robot.launcher.getFlywheelVelocity(),
                robot.launcher.getTargetVelocity()));
        panels.addData("Gate", robot.gate.isOpen() ? "open" : "closed");
        panels.addLine(recorder.isRecording() ? "square: stop this clip" : "square: start a new clip");
        panels.update(telemetry);
    }

    /** Intake and shooting, on TeleOp's buttons. */
    private void bindControls() {
        if (gamepad1.triangleWasPressed()) schedule(robot.intake.toggleIntake());
        if (gamepad1.crossWasPressed()) schedule(robot.intake.toggleOuttake());

        if (gamepad1.rightBumperWasPressed()) schedule(robot.launcher.spinUp());
        if (gamepad1.leftBumperWasPressed()) schedule(robot.launcher.idle());

        if (gamepad1.rightTriggerWasPressed()) schedule(robot.startShooting());
        if (gamepad1.rightTriggerWasReleased()) schedule(robot.stopShooting());
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
        robot.stop();
        portal.close();
    }
}
