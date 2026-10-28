package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drivetrain, localizer and path-following config. Each block has the same shape the Pedro tuners print,
 * so paste their output over it. Run them in order (see Tuning): Mecanum Tuner, Pinpoint Tuner, Foresight Tuner.
 */
public class Constants {
    // goBILDA 312 RPM mecanum
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.backLeftName.set("leftRear");
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Offsets measured on the robot: keep them if you run the Pinpoint Tuner for the pod directions
        c.xPodOffset.set(5.0);
        c.yPodOffset.set(4.8);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO: paste the Foresight Tuner output over this. Placeholders until then (Oct 26), estimated for 312 RPM motors
    // on 104 mm mecanum wheels with the tuner's own formulas, from: about 57 in/s forward and 46 in/s strafe flat out,
    // a 0.2 s speed-up time constant, 45 / 60 in/s² coasting to a stop, and turning about a point ~12 in from the
    // wheels. The kP values are half what those numbers give, so a wrong guess is sluggish rather than shaky.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27232908107134046);
                Controller secondaryTranslationalForward = Controller.proportional(0.10061831868879594);
                Controller primaryTranslationalLateral = Controller.proportional(0.4593751113521609);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1697268289155812);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010290518588093182));
                c.brake.set(Controller.proportionalFeedforward(0.008746940799879204));

                c.headingFeedback.set(Controller.proportional(4.008601635578677));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05119995476703657, 0.005495822160256111));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09240553278662739, 0.08164467944008973));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015046844948928347, 0.0015172981998579245));

                c.maxAchievableForwardVelocity.set(90.74333815072183);
                c.maxAchievableStrafeVelocity.set(75.81656408044174);
                c.naturalForwardDeceleration.set(31.90180721580304);
                c.naturalStrafeDeceleration.set(67.51615368616619);
            }
    );

    public static Mecanum drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static PinpointLocalizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Foresight foresight() {
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), foresight());
    }
}
