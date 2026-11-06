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
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Robot;

public class KhoiRoutine extends AutoRoutine {
    private final PoseFactory poseFactory = PoseFactory.degrees();



    @Override
    public Pose startPose() {
        return start;
    }



    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(36.8733, 27.1308, 90);
    private final Pose point2 = poseFactory.of(8.6085, 8.7411, -90);
    private final Pose point3Start = poseFactory.of(8.6085, 8.7411, -90);
    private final Pose point3 = poseFactory.of(59.3007, 22.4622, 90);
    private final Pose point4 = poseFactory.of(12.6858, 46.5453, 180);
    private final Pose point5 = poseFactory.of(54.5406, 111.7869, 180);
    private final Pose point5Control1 = poseFactory.of(9.3348, 92.9362, 0);
    private final Pose point6 = poseFactory.of(49.1041, 128.3194, 108.2027);
    private final Pose point7 = poseFactory.of(49.0731, 111.1806, 89.8963);
    private final Pose point8 = poseFactory.of(6.6815, 106.8668, 5.8104);

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).constant(point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }
    public KhoiRoutine(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }
    @Override
    public Command autoRoutine() {
        Command auto = sequential(
                robot.shoot(),
                follow(follower, path1()),
                follow(follower,path2()),

                robot.intake.intake(),
                waitMs(300),
                robot.intake.off(),
                follow(follower,path3()),
                robot.shoot(),
                follow(follower,path4()),
                robot.intake.intake(),
                waitMs(300),
                robot.intake.off(),
                follow(follower,path5()),
                robot.flipHive(),
                robot.shoot(),
                follow(follower,path6()),
                robot.intake.intake(),
                waitMs(300),
                robot.intake.off(),
                follow(follower,path7()),
                robot.shoot(),
                follow(follower,path8())


        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(auto, robot.aimAtHive(), robot.enableFlywheel()),
                robot.launcher.idle(),
                follow(follower, path8())
        );
    }
}