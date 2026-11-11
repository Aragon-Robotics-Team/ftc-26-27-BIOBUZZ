package org.firstinspires.ftc.teamcode.routines;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Robot;

/**
 * 1 Cycle + Garden + Flowers + Park: {@link OneCycleGardenPark}, then instead of parking straight away, knock the
 * pollen out of the alliance-wall flower and the rear-wall flower by sliding back and forth against each, then park
 * on the top side of the loading zone with the intake still running.
 */
public class OneCycleGardenParkWithFlowers extends AutoRoutine {
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

    private Path toAllianceFlower() {
        return line(poses.adjustToPark, poses.allianceFlowerApproach).tangent();
    }

    private Path allianceFlowerIn() {
        return line(poses.allianceFlowerApproach, poses.allianceFlowerLow).linear(poses.allianceFlowerApproach, poses.allianceFlowerLow);
    }

    private Path allianceFlowerUp() {
        return line(poses.allianceFlowerLow, poses.allianceFlowerHigh).linear(poses.allianceFlowerLow, poses.allianceFlowerHigh);
    }

    private Path allianceFlowerDown() {
        return line(poses.allianceFlowerHigh, poses.allianceFlowerLow).linear(poses.allianceFlowerHigh, poses.allianceFlowerLow);
    }

    private Path allianceFlowerOut() {
        return line(poses.allianceFlowerLow, poses.allianceFlowerBackOff).linear(poses.allianceFlowerLow, poses.allianceFlowerBackOff);
    }

    private Path toRearFlower() {
        return line(poses.allianceFlowerBackOff, poses.rearFlowerApproach).tangent();
    }

    private Path rearFlowerIn() {
        return line(poses.rearFlowerApproach, poses.rearFlowerLeft).linear(poses.rearFlowerApproach, poses.rearFlowerLeft);
    }

    private Path rearFlowerSlideRight() {
        return line(poses.rearFlowerLeft, poses.rearFlowerRight).linear(poses.rearFlowerLeft, poses.rearFlowerRight);
    }

    private Path rearFlowerSlideLeft() {
        return line(poses.rearFlowerRight, poses.rearFlowerLeft).linear(poses.rearFlowerRight, poses.rearFlowerLeft);
    }

    private Path rearFlowerOut() {
        return line(poses.rearFlowerRight, poses.rearFlowerBackOff).linear(poses.rearFlowerRight, poses.rearFlowerBackOff);
    }

    private Path park() {
        return line(poses.rearFlowerBackOff, poses.topPark).linear(poses.rearFlowerBackOff, poses.topPark);
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
                follow(follower, adjustBeforePark()),

                // Alliance-wall flower: slide along the wall and back to knock its pollen out (intake still on)
                follow(follower, toAllianceFlower()),
                follow(follower, allianceFlowerIn()),
                follow(follower, allianceFlowerUp()),
                follow(follower, allianceFlowerDown()),
                follow(follower, allianceFlowerOut()),

                // Rear-wall flower: slide back and forth three times to get all of its pollen out
                follow(follower, toRearFlower()),
                follow(follower, rearFlowerIn()),
                follow(follower, rearFlowerSlideRight()),
                follow(follower, rearFlowerSlideLeft()),
                follow(follower, rearFlowerSlideRight()),
                follow(follower, rearFlowerSlideLeft()),
                follow(follower, rearFlowerSlideRight()),
                follow(follower, rearFlowerOut())
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
