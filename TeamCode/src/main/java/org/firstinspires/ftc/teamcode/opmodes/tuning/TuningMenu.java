package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Edit numbers live from a gamepad.
 * Dpad up/down picks a value, dpad left/right changes it, bumpers make the step 10x smaller / bigger.
 * Changes only last until the OpMode stops, so copy the final values into RobotConstants.
 */
public class TuningMenu {
    private static class Param {
        final String name;
        final DoubleSupplier get;
        final DoubleConsumer set;
        double step;

        Param(String name, double step, DoubleSupplier get, DoubleConsumer set) {
            this.name = name;
            this.step = step;
            this.get = get;
            this.set = set;
        }
    }

    private final List<Param> params = new ArrayList<>();
    private int selected = 0;

    public TuningMenu add(String name, double step, DoubleSupplier get, DoubleConsumer set) {
        params.add(new Param(name, step, get, set));
        return this;
    }

    /** Read the gamepad once per loop. Returns true if a value changed. */
    public boolean update(Gamepad gamepad) {
        if (gamepad.dpadDownWasPressed()) selected = (selected + 1) % params.size();
        if (gamepad.dpadUpWasPressed()) selected = (selected + params.size() - 1) % params.size();

        Param param = params.get(selected);
        if (gamepad.rightBumperWasPressed()) param.step *= 10;
        if (gamepad.leftBumperWasPressed()) param.step /= 10;

        double delta = 0;
        if (gamepad.dpadRightWasPressed()) delta += param.step;
        if (gamepad.dpadLeftWasPressed()) delta -= param.step;
        if (delta == 0) return false;
        param.set.accept(param.get.getAsDouble() + delta);
        return true;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Dpad ↑↓ select, ←→ change, bumpers step ÷10 / ×10");
        for (int i = 0; i < params.size(); i++) {
            Param param = params.get(i);
            telemetry.addData((i == selected ? "> " : "  ") + param.name, "%.5g  (step %.3g)",
                    param.get.getAsDouble(), param.step);
        }
    }
}
