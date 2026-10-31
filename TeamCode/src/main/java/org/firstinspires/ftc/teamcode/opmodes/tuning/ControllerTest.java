package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

/**
 * Shows everything the robot reads from both gamepads: the controller type, every stick and trigger value, and
 * which buttons are down. Works from INIT, no need to press start. Use it to check a controller's mapping: each
 * stick should move only its own two values, with up = -y and right = +x.
 * Button names are Xbox / PlayStation.
 */
@TeleOp(name = "Controller Test", group = "Tuning")
public class ControllerTest extends OpMode {
    private TelemetryManager panels;

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        show();
    }

    @Override
    public void loop() {
        show();
    }

    private void show() {
        show("Gamepad 1", gamepad1);
        show("Gamepad 2", gamepad2);
        panels.update(telemetry);
    }

    private void show(String name, Gamepad g) {
        panels.addLine(name + ": " + g.type);
        panels.addData(name + " left stick x", g.left_stick_x);
        panels.addData(name + " left stick y", g.left_stick_y);
        panels.addData(name + " right stick x", g.right_stick_x);
        panels.addData(name + " right stick y", g.right_stick_y);
        panels.addData(name + " left trigger", g.left_trigger);
        panels.addData(name + " right trigger", g.right_trigger);

        List<String> down = new ArrayList<>();
        if (g.a) down.add("A/cross");
        if (g.b) down.add("B/circle");
        if (g.x) down.add("X/square");
        if (g.y) down.add("Y/triangle");
        if (g.dpad_up) down.add("dpad up");
        if (g.dpad_down) down.add("dpad down");
        if (g.dpad_left) down.add("dpad left");
        if (g.dpad_right) down.add("dpad right");
        if (g.left_bumper) down.add("L1/LB");
        if (g.right_bumper) down.add("R1/RB");
        if (g.left_stick_button) down.add("left stick button");
        if (g.right_stick_button) down.add("right stick button");
        if (g.back) down.add("back/share");
        if (g.start) down.add("start/options");
        if (g.guide) down.add("guide/PS");
        if (g.touchpad) down.add("touchpad");
        panels.addLine(name + " buttons: " + (down.isEmpty() ? "none" : String.join(", ", down)));
        panels.addLine("");
    }
}
