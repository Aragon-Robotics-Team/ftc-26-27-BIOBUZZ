package org.firstinspires.ftc.teamcode.keyboard;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

/**
 * The SDK's own telemetry, which also hands every update to the keyboard page. With no Driver Station connected the
 * SDK drops telemetry, so without this the page couldn't show what the OpMode prints. KeyboardBridge swaps it into
 * each OpMode just before init(); it is still a TelemetryImpl, so the SDK treats it exactly like the original.
 */
final class MirrorTelemetry extends TelemetryImpl {
    interface Sink {
        void telemetry(List<String> lines);
        void displayFormat(DisplayFormat format);
    }

    private final Sink sink;

    MirrorTelemetry(OpMode opMode, Sink sink) {
        super(opMode);
        this.sink = sink;
    }

    @Override
    protected void saveToTransmitter(boolean recompose, TelemetryMessage message) {
        super.saveToTransmitter(recompose, message);
        // Copy now: the SDK clears the message once it is sent. Keys sort in line order, log entries last.
        sink.telemetry(new ArrayList<>(new TreeMap<>(message.getDataStrings()).values()));
    }

    @Override
    public void setDisplayFormat(DisplayFormat format) {
        sink.displayFormat(format);
        super.setDisplayFormat(format);
    }
}
