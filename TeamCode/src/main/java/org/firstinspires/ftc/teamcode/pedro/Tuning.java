package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/** Pedro tuning procedures, in the order to run them. Each one prints config to paste into {@link Constants}. */
public class Tuning {
    @Tuner(name = "1. Mecanum Tuner")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint Tuner")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight Tuner")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    @Tuner(name = "4. Tests")
    public static Procedure tests() {
        return new Tests(Constants::drivetrain, Constants::localizer, Constants::foresight);
    }
}
