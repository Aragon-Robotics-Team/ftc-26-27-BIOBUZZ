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
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.0);
        c.yPodOffset.set(4.8);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO: paste the Foresight Tuner output over this. Placeholders until then (Oct 26), estimated for 312 RPM motors
    // on 104 mm mecanum wheels with the tuner's own formulas, from: about 57 in/s forward and 46 in/s strafe flat out,
    // a 0.2 s speed-up time constant, 45 / 60 in/s² coasting to a stop, and turning about a point ~12 in from the
    // wheels. The kP values are half what those numbers give, so a wrong guess is sluggish rather than shaky.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18);
                Controller secondaryTranslationalForward = Controller.proportional(0.067);
                Controller primaryTranslationalLateral = Controller.proportional(0.23);
                Controller secondaryTranslationalLateral = Controller.proportional(0.084);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0175));
                c.brake.set(Controller.proportionalFeedforward(0.0149));

                c.headingFeedback.set(Controller.proportional(5.0));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03, 0.13));

                c.linearBrakeCoefficients.set(Matrix.diag(0.03, 0.03));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.011, 0.0083));

                c.maxAchievableForwardVelocity.set(57.0);
                c.maxAchievableStrafeVelocity.set(46.0);
                c.naturalForwardDeceleration.set(45.0);
                c.naturalStrafeDeceleration.set(60.0);
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
