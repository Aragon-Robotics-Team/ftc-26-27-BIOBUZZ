package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Common lifecycle every subsystem implements. Call these once per loop from the OpMode. */
public interface Subsystem {
    /** Push the current target state to hardware. */
    void update();

    /** Put the subsystem into a safe, powered-down state. */
    void stop();

    /** Report subsystem state to the driver station. */
    default void telemetry(Telemetry telemetry) {}
}
