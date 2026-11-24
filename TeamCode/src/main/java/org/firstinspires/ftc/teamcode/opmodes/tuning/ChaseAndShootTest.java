package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Ball Chase Test, over and over: chase, drive to the launch spot and shoot, then chase again from there, until a
 * chase picks up nothing or there's no shot from the launch spot. Everything else (init, settings under Configurables
 * > BallChaseTest, telemetry, the log) is Ball Chase Test's. A test, not a menu Auto.
 */
@Autonomous(name = "Chase And Shoot Test", group = "Tuning")
public class ChaseAndShootTest extends BallChaseTest {
    @Override
    protected boolean cycles() {
        return true;
    }
}
