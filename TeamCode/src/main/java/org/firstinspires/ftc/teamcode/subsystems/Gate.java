package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.RobotConstants.Gate.CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.Gate.OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.RobotConstants.Gate.SERVO_NAME;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Stop between storage and the flywheel. Open it while the intake runs and everything stored goes to the shooter. */
public class Gate implements Subsystem {
    private final Servo servo;
    private boolean open = false;

    public Gate(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
    }

    // ---- Commands ----

    public Command open() {
        return set(true);
    }

    public Command close() {
        return set(false);
    }

    private Command set(boolean open) {
        return instant(() -> {
            this.open = open;
            servo.setPosition(open ? OPEN_POSITION : CLOSED_POSITION);
        }).requiring(servo);
    }

    public boolean isOpen() {
        return open;
    }

    @Override
    public void start() {
        servo.setPosition(CLOSED_POSITION);
    }

    @Override
    public void stop() {
        // Servos hold their last position; leave it
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Gate", open ? "OPEN" : "CLOSED");
    }
}
