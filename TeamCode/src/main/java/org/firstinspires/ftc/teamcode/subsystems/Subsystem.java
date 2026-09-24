package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Common lifecycle every subsystem implements. The OpMode drives these through {@link org.firstinspires.ftc.teamcode.Robot}. */
public interface Subsystem {
    /** Called once when the match starts, before the first update(). */
    default void start() {}

    /** Push the current target state to hardware. */
    void update();

    /** Put the subsystem into a safe, powered-down state. */
    void stop();

    /** Report subsystem state to the driver station. */
    default void telemetry(Telemetry telemetry) {}
}
