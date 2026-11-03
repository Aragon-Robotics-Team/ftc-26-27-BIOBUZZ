package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

public class CeciliaRoutine extends AutoRoutine {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 180);

    @Override
    public Pose startPose() {
        return start;
    }

    private final Pose path1 = poseFactory.of(8.5457, 8.8216, 180);
    private final Pose point2Start = poseFactory.of(8.5457, 8.8216, 180);
    private final Pose point2 = poseFactory.of(54.3529, 27.782, 90);
    private final Pose point3 = poseFactory.of(12.4276, 98.8643, 90);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }
    public CeciliaRoutine(Robot robot, Alliance alliance) {
            super(robot, alliance);
    }

    @Override
    public Command autoRoutine() {
        Command auto = sequential(
                follow(follower, path1()),
                robot.shoot(),
                // The preloads tip the hive, so the other cell is up now
                robot.flipHive(),

                // Run the intake on the way to the pickup, stop it on the way back
                robot.intake.intake(),
                follow(follower, path2() ),
                waitMs(300),
                robot.intake.off(),
                follow(follower, path3() ),
                robot.shoot()
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(auto, robot.aimAtHive(), robot.enableFlywheel()),
                robot.launcher.idle(),
                follow(follower, path3())
        );
    }

}



