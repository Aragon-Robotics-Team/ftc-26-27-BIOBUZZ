package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Tune the twin stick heading PID with Panels: edit the values under Configurables > HeadingTuner and watch
 * target / actual in the Graph. Copy the final values into RobotConstants.Drive.
 * The robot starts at heading 0, so stick forward is the way it faces at start.
 * 1. Raise kP until the robot turns to the target quickly, then back off if it overshoots.
 * 2. Add kD to damp any overshoot. Add kI only if it stops just short of the target.
 * X / A / B (gamepad 1) jump the target left / center / right by stepDegrees, Y turns it around (the hardest case).
 * The sticks drive like TeleOp's twin stick drive, to check it holds the heading while driving.
 */
@Configurable
@TeleOp(name = "Heading Tuner", group = "Tuning")
public class HeadingTuner extends OpMode {
    public static double stepDegrees = 90;
    public static double kP = RobotConstants.Drive.HEADING_KP;
    public static double kI = RobotConstants.Drive.HEADING_KI;
    public static double kD = RobotConstants.Drive.HEADING_KD;
    public static double maxPower = RobotConstants.Drive.HEADING_MAX_POWER;

    private Drivetrain drivetrain;
    private TelemetryManager panels;

    @Override
    public void init() {
        Scheduler.reset();

        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setPose(new Pose(0, 0, 0));
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        drivetrain.start();
        schedule(drivetrain.driveTwinStick(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_y,
                () -> -gamepad1.right_stick_x,
                0));
    }

    @Override
    public void loop() {
        // The drivetrain reads these every loop, so edits in Panels take effect right away
        RobotConstants.Drive.HEADING_KP = kP;
        RobotConstants.Drive.HEADING_KI = kI;
        RobotConstants.Drive.HEADING_KD = kD;
        RobotConstants.Drive.HEADING_MAX_POWER = maxPower;

        // Positive heading is counter-clockwise, i.e. to the left
        if (gamepad1.xWasPressed()) drivetrain.setTargetHeading(Math.toRadians(stepDegrees));
        if (gamepad1.aWasPressed()) drivetrain.setTargetHeading(0);
        if (gamepad1.bWasPressed()) drivetrain.setTargetHeading(Math.toRadians(-stepDegrees));
        if (gamepad1.yWasPressed()) drivetrain.setTargetHeading(drivetrain.getTargetHeading() + Math.PI);

        drivetrain.update();
        Scheduler.execute();

        double target = Math.toDegrees(drivetrain.getTargetHeading());
        double actual = Math.toDegrees(Angle.normalizeSigned(drivetrain.getPose().heading()));
        panels.addLine("X left, A center, B right, Y turn around");
        panels.addData("Target (deg)", target);
        panels.addData("Actual (deg)", actual);
        panels.addData("Error (deg)", Math.toDegrees(Angle.normalizeSigned(Math.toRadians(target - actual))));
        panels.addData("Turn speed (deg/s)", Math.toDegrees(drivetrain.getVelocity().omega));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        Scheduler.reset();
        drivetrain.stop();
    }
}
