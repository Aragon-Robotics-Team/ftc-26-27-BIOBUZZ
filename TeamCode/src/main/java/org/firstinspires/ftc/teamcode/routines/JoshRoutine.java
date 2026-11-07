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

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Robot;

public class JoshRoutine extends AutoRoutine {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(10, 20, 90);

    @Override
    public Pose startPose() {
        return start;
    }

    private final Pose point1 = poseFactory.of(10, 10, -90);
    private final Pose point2 = poseFactory.of(58, 33, 25.6022);
    private final Pose point3Start = poseFactory.of(58, 33, 90);
    private final Pose point3 = poseFactory.of(58, 111, 90);
    private final Pose point4 = poseFactory.of(50, 130, 112.8337);
    private final Pose point5 = poseFactory.of(9.9348, 116.4329, -161.2926);

    public Path path1() {return Paths.line(start, point1).tangent();}

    public Path path2() {return Paths.line(point1, point2).tangent();}

    public Path path3() {return Paths.line(point3Start, point3).linear(point3Start, point3);}

    public Path path4() {return Paths.line(point3, point4).tangent();}

    public Path path5() {return Paths.line(point4, point5).tangent();}
    public JoshRoutine(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }
    @Override
    public Command autoRoutine() {
        Command auto = sequential(
                robot.shoot(),
                robot.flipHive(),
                robot.intake.intake(),
                follow(follower, path1()),
                waitMs(300),
                robot.intake.off(),
                follow(follower, path2()),
                follow(follower, path3()),
                robot.shoot(),
                robot.intake.intake(),
                follow(follower, path4()),
                waitMs(300),
                robot.intake.off(),
                robot.shoot(),
                robot.flipHive()
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(auto, robot.aimAtHive(), robot.enableFlywheel()),
                robot.launcher.idle(),
                follow(follower, path5())
        );
    }
}
