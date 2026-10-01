package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * One autonomous routine. The Auto OpMode lets the drivers pick one from {@link Routines}, then runs it.
 *
 * Write every pose for the red alliance with {@link #poseFactory} (degrees). It rotates them 180° automatically
 * when blue is selected, so the same routine runs on both sides. Field locations several routines use live in
 * {@link FieldPoses}, available as {@link #poses}.
 *
 * To add an auto: copy an existing routine, change its poses / paths / autoRoutine(), and add it to {@link Routines}.
 */
public abstract class AutoRoutine {
    protected final Robot robot;
    protected final Follower follower;
    protected final Alliance alliance;
    protected final PoseFactory poseFactory;
    protected final FieldPoses poses;

    protected AutoRoutine(Robot robot, Alliance alliance) {
        this.robot = robot;
        this.follower = robot.drivebase.getFollower();
        this.alliance = alliance;
        this.poseFactory = PoseFactory.degrees().map(alliance::apply);
        this.poses = new FieldPoses(poseFactory);
    }

    /** Where the robot is placed at the start of the match. */
    public abstract Pose startPose();

    /** The whole auto as one command. */
    public abstract Command autoRoutine();
}
