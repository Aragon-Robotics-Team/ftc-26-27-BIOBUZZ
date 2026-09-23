package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {
    public enum State {
        OFF,
        INTAKING,
        OUTTAKING
    }

    // TODO: tune
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;

    private final DcMotorEx motor;
    private State state = State.OFF;

    public Intake(HardwareMap hardwareMap) {
        // TODO: match the name in the robot configuration, set direction / zero power behavior
        motor = hardwareMap.get(DcMotorEx.class, "intake");
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
        // TODO: apply power for the current state
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
