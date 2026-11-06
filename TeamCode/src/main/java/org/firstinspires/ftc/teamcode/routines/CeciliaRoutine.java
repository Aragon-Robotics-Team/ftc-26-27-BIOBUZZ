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

public class CeciliaRoutine extends AutoRoutine {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 180);

    @Override
    public Pose startPose() {
        return start;
    }



    private final Pose path1 = poseFactory.of(29.5152, 23.9285, 180);
    private final Pose path2Start = poseFactory.of(29.5152, 23.9285, 180);
    private final Pose path2 = poseFactory.of(8.1407, 10.144, 270);
    private final Pose path3Start = poseFactory.of(8.1407, 10.144, 0);
    private final Pose path3 = poseFactory.of(58.7655, 30.0794, 90);
    private final Pose path4Start = poseFactory.of(58.7655, 30.0794, 180);
    private final Pose path4 = poseFactory.of(12.2961, 47.3394, 180);
    private final Pose path5Start = poseFactory.of(12.2961, 47.3394, 180);
    private final Pose path5 = poseFactory.of(54.6833, 112.3942, 270);
    private final Pose path5Control1 = poseFactory.of(22.7898, 108.9377, 0);
    private final Pose path6Start = poseFactory.of(54.6833, 112.3942, 270);
    private final Pose path6 = poseFactory.of(46.8544, 129.506, 90);
    private final Pose path7Start = poseFactory.of(46.8544, 129.506, 90);
    private final Pose path7 = poseFactory.of(53.5736, 106.8318, 270);
    private final Pose path8 = poseFactory.of(6.425, 98.7993, -170.3316);


    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path2Start, path2).linear(path2Start, path2);
    }

    public Path path3() {
        return Paths.line(path3Start, path3).linear(path3Start, path3);
    }

    public Path path4() {
        return Paths.line(path4Start, path4).linear(path4Start, path4);
    }

    public Path path5() {
        return Paths.curve(path5Start, path5Control1, path5).linear(path5Start, path5);
    }

    public Path path6() {
        return Paths.line(path6Start, path6).linear(path6Start, path6);
    }

    public Path path7() {
        return Paths.line(path7Start, path7).linear(path7Start, path7);
    }

    public Path path8() {
        return Paths.line(path7, path8).tangent();
    }
    public CeciliaRoutine(Robot robot, Alliance alliance) {
            super(robot, alliance);
    }

    @Override
    public Command autoRoutine() {
        Command auto = sequential(
                follow(follower, path1()),
                robot.intake.intake(),
                follow(follower,path2()),
                waitMs(300),
                robot.intake.off(),
                follow(follower,path3()),
                robot.shoot(),
                // The preloads tip the hive, so the other cell is up now
                robot.flipHive(),
                robot.intake.intake(),
                follow(follower,path4()),
                waitMs(300),
                robot.intake.off(),
                follow(follower,path5()),
                robot.shoot(),
                robot.intake.intake(),
                follow(follower,path6()),
                waitMs(300),
                robot.intake.off(),
                follow(follower,path7())
        );



        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(auto, robot.aimAtHive(), robot.enableFlywheel()),
                robot.launcher.idle(),
                follow(follower, path8())
        );
    }

}