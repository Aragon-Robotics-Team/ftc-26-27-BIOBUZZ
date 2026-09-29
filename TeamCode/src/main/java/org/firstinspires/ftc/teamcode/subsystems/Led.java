package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static org.firstinspires.ftc.teamcode.RobotConstants.Led.FLASH_MS;
import static org.firstinspires.ftc.teamcode.RobotConstants.Led.SERVO_NAME;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * goBILDA RGB Indicator Light (PWM). Shows whether a shot right now would go in the hive:
 * off = flywheel off, green = ready to shoot (there's a shot, flywheel at speed, turret on target),
 * yellow = spinning up or aiming, red = no shot from here.
 * flash() shows white briefly to confirm a button press.
 */
public class Led implements Subsystem {
    // Servo positions from goBILDA's chart (default 600-2400 us servo range). Colors blend in between.
    private static final double OFF = 0.0;
    private static final double RED = 0.28; // chart says 0.277, which sits right on the "off" cutoff
    private static final double YELLOW = 0.388;
    private static final double GREEN = 0.5;
    private static final double WHITE = 1.0;

    private final Servo light;
    private final Launcher launcher;
    private final ElapsedTime flashTimer = new ElapsedTime();
    private boolean flashing = false;

    public Led(HardwareMap hardwareMap, Launcher launcher) {
        light = hardwareMap.get(Servo.class, SERVO_NAME);
        this.launcher = launcher;
    }

    // ---- Commands ----

    /** Show white for a moment, e.g. after the hive target changes. */
    public Command flash() {
        return instant(() -> {
            flashing = true;
            flashTimer.reset();
        });
    }

    @Override
    public void update() {
        if (flashing && flashTimer.milliseconds() > FLASH_MS) flashing = false;
        light.setPosition(color());
    }

    private double color() {
        if (flashing) return WHITE;
        switch (launcher.getStatus()) {
            case READY: return GREEN;
            case NO_SHOT: return RED;
            case SPINNING_UP:
            case AIMING: return YELLOW;
            default: return OFF;
        }
    }

    @Override
    public void stop() {
        light.setPosition(OFF);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("LED position", "%.3f", light.getPosition());
    }
}
