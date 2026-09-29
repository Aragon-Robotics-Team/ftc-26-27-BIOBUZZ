package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.RobotConstants.Intake.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.RobotConstants.Intake.MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.RobotConstants.Intake.OUTTAKE_POWER;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// TODO: match the real intake
public class Intake implements Subsystem {
    private final DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        // TODO: set direction + zero power behavior
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
    }

    // ---- Commands ----

    public Command intake() {
        return setPower(INTAKE_POWER);
    }

    public Command outtake() {
        return setPower(OUTTAKE_POWER);
    }

    public Command off() {
        return setPower(0);
    }

    private Command setPower(double power) {
        return instant(() -> motor.setPower(power)).requiring(motor);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake power", "%.2f", motor.getPower());
    }
}
