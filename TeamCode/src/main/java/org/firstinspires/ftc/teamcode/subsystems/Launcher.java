package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher implements Subsystem {
    public enum State {
        IDLE,
        SPINNING_UP,
        READY
    }

    // TODO: tune
    public static double TARGET_VELOCITY = 0;     // ticks/sec
    public static double VELOCITY_TOLERANCE = 50; // ticks/sec

    private final DcMotorEx motor;
    private State state = State.IDLE;
    private double targetVelocity = 0;

    public Launcher(HardwareMap hardwareMap) {
        // TODO: match the name in the robot configuration, set direction / run mode / PIDF
        motor = hardwareMap.get(DcMotorEx.class, "launcher");
    }

    public void spinUp() {
        spinUp(TARGET_VELOCITY);
    }

    public void spinUp(double velocity) {
        targetVelocity = velocity;
        state = State.SPINNING_UP;
    }

    public void idle() {
        targetVelocity = 0;
        state = State.IDLE;
    }

    /** Fire a game piece. Only meaningful once {@link #isReady()}. */
    public void launch() {
        // TODO: trigger feeder / gate
    }

    public boolean isReady() {
        return state == State.READY;
    }

    public State getState() {
        return state;
    }

    @Override
    public void update() {
        // TODO: command targetVelocity, move SPINNING_UP -> READY when within VELOCITY_TOLERANCE
    }

    @Override
    public void stop() {
        idle();
        motor.setPower(0);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Launcher", state);
        telemetry.addData("Launcher target / actual", "%.0f / %.0f", targetVelocity, motor.getVelocity());
    }
}
