package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.Intake.MOTOR_NAME;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// TODO: match the real intake
public class Intake implements Subsystem {
    public enum State {
        OFF,
        INTAKING,
        OUTTAKING
    }

    private final DcMotorEx motor;
    private State state = State.OFF;

    public Intake(HardwareMap hardwareMap) {
        // TODO: set direction + zero power behavior
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
    }

    public void intake() {
        state = State.INTAKING;
    }

    public void outtake() {
        state = State.OUTTAKING;
    }

    public void off() {
        state = State.OFF;
    }

    public State getState() {
        return state;
    }

    @Override
    public void update() {
        // TODO: set power for state
    }

    @Override
    public void stop() {
        off();
        motor.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake", state);
    }
}
