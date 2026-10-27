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

import org.firstinspires.ftc.teamcode.subsystems.BallVision;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.firstinspires.ftc.teamcode.vision.BallGeometry;

import java.util.List;

/**
 * Check the ball camera and its mount. Edit the values under Configurables > BallVisionTest in Panels; copy the final
 * ones into RobotConstants.BallVision.
 *
 * Every ball in the newest frame is listed with where the camera model puts it (inches from the robot center: forward,
 * left) and its size against the size the model expects there. The circles are drawn on the Driver Station camera
 * stream and the Panels camera stream.
 *
 * Gamepad 1 drives (field-centric from where the robot faces at start, like the Heading Tuner): left stick drives,
 * right stick x turns, hold the left trigger for slow mode. Positions are relative to the robot, so they follow it, but
 * only measure (step 2) with the robot stopped: the newest frame lags the robot a little.
 * 1. Exposure and gain apply right away. Raise exposure until the balls are found reliably; keep it as short as that
 *    allows (motion blur while driving: drive past balls to check).
 * 2. Mount: put a ball at a tape-measured spot on the tiles, in front of the robot. Its forward / left should match.
 *    Change the mount values and press cross to apply them (the camera reopens, about a second).
 *    - Every ball reads too far or too near: lensHeight or tiltDegrees is off (tilt matters most far away).
 *    - Every ball is swung to the same side, more the further out it is: yawDegrees.
 *    - The error grows from one side of the image to the other: rollDegrees (try flipping its sign).
 *    - size about 1.0 everywhere means the size gate is centered on real balls (it allows 0.55-1.60).
 */
@Configurable
@TeleOp(name = "Ball Vision Test", group = "Tuning")
public class BallVisionTest extends OpMode {
    public static double exposureMs = EXPOSURE_MS;
    public static int gain = GAIN;
    public static double lensForward = LENS_FORWARD;
    public static double lensLeft = LENS_LEFT;
    public static double lensHeight = LENS_HEIGHT;
    public static double yawDegrees = Math.toDegrees(YAW);
    public static double tiltDegrees = Math.toDegrees(TILT);
    public static double rollDegrees = Math.toDegrees(ROLL);

    private Drivetrain drivetrain;
    private BallVision vision;
    private TelemetryManager panels;

    @Override
    public void init() {
        Scheduler.reset();
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setPose(new Pose(0, 0, 0));
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        open();
    }

    private void open() {
        LENS_FORWARD = lensForward;
        LENS_LEFT = lensLeft;
        LENS_HEIGHT = lensHeight;
        YAW = Math.toRadians(yawDegrees);
        TILT = Math.toRadians(tiltDegrees);
        ROLL = Math.toRadians(rollDegrees);
        vision = new BallVision(hardwareMap, drivetrain::getPose);
        vision.enable(true);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Ball vision", vision.status());
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
    }

    private double driveScale() {
        return gamepad1.left_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
    }

    @Override
    public void loop() {
        // BallVision reads these every loop
        EXPOSURE_MS = exposureMs;
        GAIN = gain;
        if (gamepad1.crossWasPressed()) {
            vision.stop();
            open();
        }
        drivetrain.update();
        Scheduler.execute();
        vision.update();

        List<Ball> balls = vision.getBalls();
        int[] count = new int[Ball.Type.values().length];
        for (Ball ball : balls) count[ball.type.ordinal()]++;
        panels.addData("Vision", vision.status());
        panels.addData("Frames", vision.getFrameCount());
        panels.addData("Balls", String.format("%d: %d pollen, %d red nectar, %d blue nectar", balls.size(),
                count[Ball.Type.POLLEN.ordinal()], count[Ball.Type.RED_NECTAR.ordinal()],
                count[Ball.Type.BLUE_NECTAR.ordinal()]));
        panels.addData("Mount", String.format(
                "forward %.2f left %.2f height %.2f in, yaw %.1f° tilt %.1f° roll %.1f°   (cross: apply)",
                LENS_FORWARD, LENS_LEFT, LENS_HEIGHT, Math.toDegrees(YAW), Math.toDegrees(TILT), Math.toDegrees(ROLL)));

        BallGeometry geometry = vision.getGeometry();
        for (Ball ball : balls) {
            double[] p = geometry.robotPosition(ball.x, ball.y, ball.type.diameter);
            double expected = vision.getPipeline().expectedDiameter(ball.x, ball.y, ball.type.diameter);
            panels.addLine(String.format("%s: forward %s left %s in | size %.2f | pixel (%.0f, %.0f) r %.0f%s%s",
                    ball.type, p == null ? "-" : String.format("%.1f", p[0]), p == null ? "-" : String.format("%.1f", p[1]),
                    2 * ball.radius / expected, ball.x, ball.y, ball.radius,
                    ball.truncated ? " (cut off by the frame edge)" : "", ball.recovered ? " (merged with tape)" : ""));
        }
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        Scheduler.reset();
        drivetrain.stop();
        vision.stop();
    }
}
