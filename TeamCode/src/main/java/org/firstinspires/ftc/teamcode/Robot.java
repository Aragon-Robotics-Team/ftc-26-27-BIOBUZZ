package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

/** Owns every subsystem so TeleOp and Auto build the robot the same way. */
public class Robot {
    public final Drivetrain drivebase;
    public final Intake intake;
    public final Launcher launcher;

    private final Subsystem[] subsystems;

    public Robot(HardwareMap hardwareMap) {
        drivebase = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);

        subsystems = new Subsystem[]{drivebase, intake, launcher};
    }

    public void update() {
        for (Subsystem subsystem : subsystems) subsystem.update();
    }

    public void stop() {
        for (Subsystem subsystem : subsystems) subsystem.stop();
    }

    public void telemetry(Telemetry telemetry) {
        for (Subsystem subsystem : subsystems) subsystem.telemetry(telemetry);
    }
}
