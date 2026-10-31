package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;

@Configurable
@TeleOp(name = "Pose Helper", group = "Tuning")
public class PoseHelper extends OpMode {

    private PinpointLocalizer localizer;
    private Mecanum drivetrain;
    private TelemetryManager panels;
    private FieldManager field;
    private ArrayList<Pose> poseList = new ArrayList<>();
    private boolean lastB = false;


    @Override
    public void init() {
        localizer = Constants.localizer(hardwareMap);
        drivetrain = Constants.drivetrain(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());
        reset();
    }

    @Override
    public void start() {
        reset();
    }

    @Override
    public void loop() {
        localizer.update();

        boolean b = gamepad1.b;
        if (b && !lastB) savePose(localizer.pose());
        lastB = b;

        if (gamepad1.a) reset();

        updateTelemetry();
    }

    @Override
    public void stop() {
        drivetrain.drive(new DrivePowers(0, 0, 0), true);
    }
    private void reset() {
        localizer.setPose(Pose.zero());
        poseList.clear();
    }
    private void savePose(Pose currentPose) {
        poseList.add(currentPose);
    }
    private void updateTelemetry() {
        panels.addLine("Current: " + localizer.pose());
        panels.addLine("");
        int i = 1;
        for (Pose pose : poseList) {
            panels.addLine("Pose " + i + ": " + pose);
            i++;
        }
        panels.update(telemetry);
    }



}
