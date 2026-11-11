package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
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

/**
 * 1 Cycle + Garden + Park (doc/auto-pathing.md, doc/autoPP/red_1cycle_park.pp): shoot the preloads, collect the
 * garden pollen, then park on the bottom side of the loading zone with it still on board.
 */
public class OneCycleGardenParkWithFlowers extends AutoRoutine {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(38.561, 24.2993, 90);
    private final Pose point1 = poseFactory.of(17, 35, 153.6048);
    private final Pose point2Start = poseFactory.of(17, 35, 180);
    private final Pose point2 = poseFactory.of(10, 35, 180);
    private final Pose point3Start = poseFactory.of(10, 35, 180);
    private final Pose point3 = poseFactory.of(10, 40, 180);
    private final Pose point4Start = poseFactory.of(10, 40, 180);
    private final Pose point4 = poseFactory.of(10, 35, 180);
    private final Pose point5 = poseFactory.of(16.3053, 40, 180);
    private final Pose point6 = poseFactory.of(34, 118.764, 77.3384);
    private final Pose point7Start = poseFactory.of(34, 118.764, 90);
    private final Pose point7 = poseFactory.of(35, 132, 90);
    private final Pose point8Start = poseFactory.of(35, 132, 90);
    private final Pose point8 = poseFactory.of(40, 132.125, 90);
    private final Pose point9Start = poseFactory.of(40, 132.125, 90);
    private final Pose point9 = poseFactory.of(35, 132, 90);
    private final Pose point10Start = poseFactory.of(35, 132, 90);
    private final Pose point10 = poseFactory.of(40, 132, 90);
    private final Pose point11Start = poseFactory.of(40, 132, 90);
    private final Pose point11 = poseFactory.of(35, 132, 90);
    private final Pose point12Start = poseFactory.of(35, 132, 90);
    private final Pose point12 = poseFactory.of(40, 132, 90);
    private final Pose point13 = poseFactory.of(40, 125, 90);
    private final Pose point14Start = poseFactory.of(40, 125, 90);
    private final Pose point14 = poseFactory.of(10, 118, 90);

    public Path path1() {
        return Paths.line(start, point1).tangent();
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }

    public Path path9() {
        return Paths.line(point8, point9).tangent();
    }

    public Path path10() {
        return Paths.line(point10Start, point10).linear(point10Start, point10);
    }

    public Path path11() {
        return Paths.line(point11Start, point11).linear(point11Start, point11);
    }

    public Path path12() {
        return Paths.line(point12Start, point12).linear(point12Start, point12);
    }

    public Path path13() {
        return Paths.line(point13Start, point13).linear(point13Start, point13);
    }

    public Path path14() {
        return Paths.line(point14Start, point14).linear(point14Start, point14);
    }

    public Path path15() {
        return Paths.line(point15Start, point15).linear(point15Start, point15);
    }

    public Path path16() {
        return Paths.line(point15, point16).reverseTangent();
    }

    public Path path17() {
        return Paths.line(point17Start, point17).linear(point17Start, point17);
    } //end of flower code


    public OneCycleGardenParkWithFlowers(Robot robot, Alliance alliance) {
        super(robot, alliance);
    }

    @Override
    public Pose startPose() {
        return poses.start;
    }

    // Path methods
    private Path startToLaunch() {
        return line(poses.start, poses.startLaunch).linear(poses.start, poses.startLaunch);
    }

    private Path prepareGardenIntake() {
        return line(poses.startLaunch, poses.gardenBorder).linear(poses.startLaunch, poses.gardenBorder);
    }

    private Path gardenIntake1() {
        return line(poses.gardenBorder, poses.gardenIntake).linear(poses.gardenBorder, poses.gardenIntake);
    }

    private Path gardenIntakeBack() {
        return line(poses.gardenIntake, poses.gardenIntakeBack).linear(poses.gardenIntake, poses.gardenIntakeBack);
    }

    private Path gardenIntake2() {
        return line(poses.gardenIntakeBack, poses.gardenIntake).linear(poses.gardenIntakeBack, poses.gardenIntake);
    }

    private Path adjustBeforePark() {
        return line(poses.gardenIntake, poses.adjustToPark).linear(poses.gardenIntake, poses.adjustToPark);
    }

    private Path park() {
        return line(poses.adjustToPark, poses.bottomPark).linear(poses.adjustToPark, poses.bottomPark);
    }

    @Override
    public Command autoRoutine() {
        Command cycle = sequential(
                // Preloads
                robot.intake.intake(),
                follow(follower, startToLaunch()),
                waitMs(2000),
                robot.shoot(),
                // Tip
                robot.flipHive(),

                // Garden: intake on the way in, stop once back out
                follow(follower, prepareGardenIntake()),
                follow(follower, gardenIntake1()).raceWith(waitMs(1500)),
                waitMs(1000),
                follow(follower, gardenIntakeBack()),
                follow(follower, gardenIntake2()).raceWith(waitMs(1500)),
                waitMs(1000),
                follow(follower, adjustBeforePark())
        );

        return sequential(
                // Keep the turret on the hive and the flywheel at the right speed while cycling
                deadline(cycle, robot.aimAtHive(), robot.enableFlywheel()),
                follow(follower, park()),
                waitMs(3000),
                robot.intake.off()
        );
    }
}
