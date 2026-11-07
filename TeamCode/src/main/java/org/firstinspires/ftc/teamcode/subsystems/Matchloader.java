package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Matchloader.DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Matchloader.SERVO_NAME;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Matchloader.UP_POSITION;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Front frame that swings down in front of the intake for the matchloader. */
public class Matchloader implements Subsystem {
    private final Servo servo;
    private boolean down = false;

    public Matchloader(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
    }

    public Command down() {return set(true);}
    public Command up() {return set(false);}

    public Command toggle() {
        return instant(() -> apply(!down)).requiring(servo);
    }

    private Command set(boolean down) {
        return instant(() -> apply(down)).requiring(servo);
    }

    private void apply(boolean down) {
        this.down = down;
        servo.setPosition(down ? DOWN_POSITION : UP_POSITION);
    }

    public boolean isDown() {return down;}

    @Override
    public void start() {apply(false);} // starts up

    @Override
    public void stop() {
        // servos hold their last position
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Matchloader", down ? "DOWN" : "UP");
    }
}
