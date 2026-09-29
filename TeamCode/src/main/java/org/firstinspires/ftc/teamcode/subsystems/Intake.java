package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.RobotConstants.Intake.DIRECTION;
import static org.firstinspires.ftc.teamcode.RobotConstants.Intake.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.RobotConstants.Intake.MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.Intake.OUTTAKE_POWER;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake rollers. The driver picks a mode (intake / outtake / off); feeding the shooter temporarily
 * runs it inward and then goes back to that mode.
 */
public class Intake implements Subsystem {
    public enum Mode {
        OFF,
        INTAKE,
        OUTTAKE
    }

    private final DcMotorEx motor;
    private Mode mode = Mode.OFF;
    private boolean feeding = false;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // ---- Commands ----

    public Command intake() {
        return setMode(Mode.INTAKE);
    }

    public Command outtake() {
        return setMode(Mode.OUTTAKE);
    }

    public Command off() {
        return setMode(Mode.OFF);
    }

    /** Intake, or off if already intaking. Switches straight over from outtaking. */
    public Command toggleIntake() {
        return instant(() -> apply(mode == Mode.INTAKE ? Mode.OFF : Mode.INTAKE)).requiring(motor);
    }

    /** Outtake, or off if already outtaking. Switches straight over from intaking. */
    public Command toggleOuttake() {
        return instant(() -> apply(mode == Mode.OUTTAKE ? Mode.OFF : Mode.OUTTAKE)).requiring(motor);
    }

    /** Push everything stored into the shooter until {@link #stopFeeding()}. The chosen mode is kept. */
    public Command feed() {
        return instant(() -> {
            feeding = true;
            motor.setPower(INTAKE_POWER);
        }).requiring(motor);
    }

    /** Stop feeding and go back to the chosen mode. */
    public Command stopFeeding() {
        return instant(() -> apply(mode)).requiring(motor);
    }

    private Command setMode(Mode mode) {
        return instant(() -> apply(mode)).requiring(motor);
    }

    private void apply(Mode mode) {
        this.mode = mode;
        feeding = false;
        motor.setPower(mode == Mode.INTAKE ? INTAKE_POWER : mode == Mode.OUTTAKE ? OUTTAKE_POWER : 0);
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isFeeding() {
        return feeding;
    }

    @Override
    public void stop() {
        apply(Mode.OFF);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake power", "%.2f", motor.getPower());
    }
}
