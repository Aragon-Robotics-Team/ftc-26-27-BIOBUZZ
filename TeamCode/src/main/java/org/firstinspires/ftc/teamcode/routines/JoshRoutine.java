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

public class JoshRoutine extends AutoRoutine {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(11, 30, 90);

    @Override
    public Pose startPose() {
        return start;
    }

    private final Pose point1 = poseFactory.of(11, 5, -90);
    private final Pose point2 = poseFactory.of(58, 35, 270);
    private final Pose point3 = poseFactory.of(58, 90, -90);
    private final Pose point4 = poseFactory.of(7, 110, 10.8384);
    private final Pose point4Control1 = poseFactory.of(48, 118, 0);

    public Path path1() {
        return Paths.line(start, point1).tangent();
    }

    public Path path2() {
        return Paths.line(point1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }
    public JoshRoutine(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }
    @Override
    public Command autoRoutine() {
        Command auto = sequential(
                robot.shoot(),
                robot.intake.intake(),
                follow(follower, path1()),
                waitMs(300),
                robot.intake.off(),
                follow(follower, path2()),
                robot.shoot(),
                robot.flipHive(),
                follow(follower, path3())
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(auto, robot.aimAtHive(), robot.enableFlywheel()),
                robot.launcher.idle(),
                follow(follower, path4())
        );
    }
}
