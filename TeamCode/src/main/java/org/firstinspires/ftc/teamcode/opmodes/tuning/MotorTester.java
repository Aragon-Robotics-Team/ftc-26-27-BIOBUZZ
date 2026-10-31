package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Test any motor or servo on its own. The list is every DC motor, CR servo and servo in the active robot
 * configuration, under the names from the Driver Station config, so new or renamed devices show up without
 * code changes. (A device that is configured but unplugged still shows up; it just won't move.)
 *
 * <p>Gamepad 1, any device: D-pad left / right picks the device.
 * <ul>
 *   <li>Motor / CR servo: hold R1 = forward, hold L1 = reverse, let go = stop. D-pad up / down = power up / down by powerStep.
 *   <li>Servo: R1 = position + step, L1 = position - step, square = go to 0.5. D-pad up / down = bigger /
 *       smaller step (0.01, 0.05, 0.1). A servo isn't moved until you first press one of these.
 * </ul>
 * In Panels (Configurables > MotorTester): power, servoPosition (type a value and the selected servo goes there),
 * servoMin / servoMax (keep a mechanism from binding).
 *
 * <p>Motors are driven directly (no encoder loop, no subsystem code). Motors the code knows use their
 * direction from RobotConstants / pedro Constants so forward matches TeleOp; any other motor runs FORWARD.
 * Encoders are reset at init, so start with the turret facing forward if you'll run TeleOp afterwards.
 */
@Configurable
@TeleOp(name = "Motor Tester", group = "Tuning")
public class MotorTester extends OpMode {
    public static double power = 0.5;
    public static double powerStep = 0.05;
    public static double servoPosition = 0.5;
    public static double servoStep = 0.05;
    public static double servoMin = 0;
    public static double servoMax = 1;

    private static final double[] SERVO_STEPS = {0.01, 0.05, 0.1};
    private static final double SERVO_HOME = 0.5;

    private enum Kind {
        MOTOR("motor"), CR_SERVO("CR servo"), SERVO("servo");

        final String label;

        Kind(String label) {
            this.label = label;
        }
    }

    private static class Device {
        final String name;
        final Kind kind;
        final HardwareDevice hardware;
        final String location;
        String note = "";
        double lastPosition = Double.NaN; // servos: last position this OpMode sent, NaN = never moved

        Device(String name, Kind kind, HardwareDevice hardware, String location) {
            this.name = name;
            this.kind = kind;
            this.hardware = hardware;
            this.location = location;
        }
    }

    private final List<Device> devices = new ArrayList<>();
    private int selected = 0;
    private double pushedServoPosition = Double.NaN;
    private TelemetryManager panels;

    @Override
    public void init() {
        Map<String, DcMotorSimple.Direction> knownDirections = knownDirections();

        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            DcMotorEx motor = (DcMotorEx) entry.getValue();
            Device device = new Device(entry.getKey(), Kind.MOTOR, motor,
                    location(motor.getController(), motor.getPortNumber()));
            DcMotorSimple.Direction direction = knownDirections.get(device.name);
            device.note = direction != null ? "direction from code: " + direction : "not in code, runs FORWARD";
            motor.setDirection(direction != null ? direction : DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            devices.add(device);
        }
        for (Map.Entry<String, CRServo> entry : hardwareMap.crservo.entrySet()) {
            CRServo servo = entry.getValue();
            devices.add(new Device(entry.getKey(), Kind.CR_SERVO, servo,
                    location(servo.getController(), servo.getPortNumber())));
        }
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            Servo servo = entry.getValue();
            devices.add(new Device(entry.getKey(), Kind.SERVO, servo,
                    location(servo.getController(), servo.getPortNumber())));
        }
        devices.sort(Comparator.comparing((Device d) -> d.name, String.CASE_INSENSITIVE_ORDER));

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        if (!devices.isEmpty()) onSelected();
    }

    /** Directions for the motors the robot code already uses, keyed by config name. */
    private static Map<String, DcMotorSimple.Direction> knownDirections() {
        Map<String, DcMotorSimple.Direction> directions = new HashMap<>();
        directions.put(RobotConstants.Intake.MOTOR_NAME, RobotConstants.Intake.DIRECTION);
        directions.put(RobotConstants.Launcher.FLYWHEEL_MOTOR_NAME, RobotConstants.Launcher.FLYWHEEL_DIRECTION);
        directions.put(RobotConstants.Launcher.TURRET_MOTOR_NAME, RobotConstants.Launcher.TURRET_DIRECTION);
        directions.put(Constants.drivetrainConfig.frontLeftName.get(), Constants.drivetrainConfig.frontLeftDirection.get());
        directions.put(Constants.drivetrainConfig.backLeftName.get(), Constants.drivetrainConfig.backLeftDirection.get());
        directions.put(Constants.drivetrainConfig.frontRightName.get(), Constants.drivetrainConfig.frontRightDirection.get());
        directions.put(Constants.drivetrainConfig.backRightName.get(), Constants.drivetrainConfig.backRightDirection.get());
        return directions;
    }

    /** "Control Hub port 2", or just "port 2" if the hub has no name in the config. */
    private String location(HardwareDevice controller, int port) {
        Set<String> names = hardwareMap.getNamesOf(controller);
        return names.isEmpty() ? "port " + port : names.iterator().next() + " port " + port;
    }

    @Override
    public void loop() {
        if (devices.isEmpty()) {
            telemetry.addLine("No motors or servos in the active robot configuration.");
            telemetry.addLine("Check the config on the Driver Station.");
            return;
        }

        if (gamepad1.dpadRightWasPressed()) select(selected + 1);
        if (gamepad1.dpadLeftWasPressed()) select(selected - 1);

        Device device = devices.get(selected);
        if (device.kind == Kind.SERVO) {
            runServo(device);
        } else {
            runSpinner(device);
        }

        showTelemetry(device);
    }

    /** Motors and CR servos: hold to spin. */
    private void runSpinner(Device device) {
        if (gamepad1.dpadUpWasPressed()) power += powerStep;
        if (gamepad1.dpadDownWasPressed()) power -= powerStep;
        power = Range.clip(power, 0, 1);

        double output = 0;
        if (gamepad1.right_bumper) output = power;
        else if (gamepad1.left_bumper) output = -power;
        setPower(device, output);
    }

    /** Position servos: step with the bumpers, or type a position in Panels. */
    private void runServo(Device device) {
        if (gamepad1.dpadUpWasPressed()) servoStep = nextStep(servoStep, 1);
        if (gamepad1.dpadDownWasPressed()) servoStep = nextStep(servoStep, -1);

        double start = Double.isNaN(device.lastPosition) ? SERVO_HOME : device.lastPosition;
        if (gamepad1.rightBumperWasPressed()) moveServo(device, start + servoStep);
        if (gamepad1.leftBumperWasPressed()) moveServo(device, start - servoStep);
        if (gamepad1.squareWasPressed()) moveServo(device, SERVO_HOME);
        if (servoPosition != pushedServoPosition) moveServo(device, servoPosition); // edited in Panels
    }

    private void moveServo(Device device, double position) {
        position = Math.max(servoMin, Math.min(servoMax, Range.clip(position, 0, 1)));
        ((Servo) device.hardware).setPosition(position);
        device.lastPosition = position;
        servoPosition = position;
        pushedServoPosition = position;
    }

    /** The next step size up (direction 1) or down (-1) from the list, staying put at the ends. */
    private static double nextStep(double current, int direction) {
        if (direction > 0) {
            for (double step : SERVO_STEPS) if (step > current + 1e-9) return step;
        } else {
            for (int i = SERVO_STEPS.length - 1; i >= 0; i--) if (SERVO_STEPS[i] < current - 1e-9) return SERVO_STEPS[i];
        }
        return current;
    }

    /** Stop whatever was spinning (servos hold where they are) and switch. */
    private void select(int index) {
        Device previous = devices.get(selected);
        if (previous.kind != Kind.SERVO) setPower(previous, 0);
        selected = Math.floorMod(index, devices.size());
        onSelected();
    }

    /** Show the selected servo's position in Panels without moving it. */
    private void onSelected() {
        Device device = devices.get(selected);
        if (device.kind != Kind.SERVO) return;
        servoPosition = Double.isNaN(device.lastPosition) ? SERVO_HOME : device.lastPosition;
        pushedServoPosition = servoPosition;
    }

    private static void setPower(Device device, double output) {
        if (device.kind == Kind.MOTOR) ((DcMotorEx) device.hardware).setPower(output);
        else if (device.kind == Kind.CR_SERVO) ((CRServo) device.hardware).setPower(output);
    }

    private void showTelemetry(Device device) {
        panels.addLine("== MOTOR TESTER (gamepad 1) ==");
        panels.addLine("D-pad LEFT / RIGHT: choose device");
        if (device.kind == Kind.SERVO) {
            panels.addLine("R1: move + step   L1: move - step");
            panels.addLine("SQUARE: go to 0.5");
            panels.addLine("D-pad UP / DOWN: bigger / smaller step");
        } else {
            panels.addLine("Hold R1: forward   Hold L1: reverse");
            panels.addLine("D-pad UP / DOWN: more / less power");
        }
        panels.addLine("");

        for (int i = 0; i < devices.size(); i++) {
            Device d = devices.get(i);
            panels.addLine((i == selected ? ">> " : "    ") + d.name + "  (" + d.kind.label + ", " + d.location + ")");
        }
        panels.addLine("");

        panels.addLine("Selected: " + device.name);
        switch (device.kind) {
            case MOTOR:
                DcMotorEx motor = (DcMotorEx) device.hardware;
                panels.addData("Power setting", String.format("%.2f", power));
                panels.addData("Output now", String.format("%.2f", motor.getPower()));
                panels.addData("Encoder position", motor.getCurrentPosition());
                panels.addData("Velocity (ticks/s)", String.format("%.0f", motor.getVelocity()));
                panels.addData("Current (A)", String.format("%.2f", motor.getCurrent(CurrentUnit.AMPS)));
                panels.addLine(device.note);
                break;
            case CR_SERVO:
                panels.addData("Power setting", String.format("%.2f", power));
                panels.addData("Output now", String.format("%.2f", ((CRServo) device.hardware).getPower()));
                break;
            case SERVO:
                panels.addData("Position", Double.isNaN(device.lastPosition)
                        ? "not moved yet (first press starts from 0.5)"
                        : String.format("%.3f", device.lastPosition));
                panels.addData("Step", String.format("%.2f", servoStep));
                panels.addData("Limits", String.format("%.2f to %.2f", servoMin, servoMax));
                break;
        }
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        for (Device d : devices) setPower(d, 0);
    }
}
