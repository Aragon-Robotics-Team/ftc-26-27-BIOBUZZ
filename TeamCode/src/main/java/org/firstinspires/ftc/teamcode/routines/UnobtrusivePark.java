package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Robot;

/**
 * Unobtrusive Park: start in the far corner (farStart) and drive straight to the top side of the
 * loading zone with the intake running, then turn the intake off. The shooter is never turned on.
 */
public class UnobtrusivePark extends AutoRoutine {

    @Override
    public Pose startPose() {
        return poses.farStart;
    }
    public Path park() {
        return Paths.line(poses.farStart, poses.topPark).linear(poses.farStart, poses.topPark);
    }

    public UnobtrusivePark(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }

    @Override
    public Command autoRoutine() {
        // No shooting: the flywheel and turret are never turned on. The intake only runs during the drive to park.
        return sequential(
                robot.launcher.idle(),
                robot.intake.intake(),
                follow(follower, park()),
                robot.intake.off()
        );
    }
}
