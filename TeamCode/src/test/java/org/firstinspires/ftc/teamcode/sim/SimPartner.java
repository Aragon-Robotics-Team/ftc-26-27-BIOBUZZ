package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.StaticRobot;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;

import java.util.Collections;

/**
 * A scripted alliance partner: an 18 in robot that drives from its wall to a spot in front of the up cell's opening
 * (which faces away from the hive, toward the rear or the audience wall), shoots its 4 preload pollen into it (each at
 * the speed that goes in), then parks in the loading zone. Balls bounce off it. It doesn't intake, and it ignores our
 * robot: the two can drive through each other.
 *
 * The script is written for red in Pedro coordinates and turned for blue with Alliance.apply, like our routines.
 */
final class SimPartner {
    private static final double SIZE = 18, HEIGHT = 14;      // inches
    private static final double SPEED = 30;                  // in/s
    private static final double TURN_RATE = Math.toRadians(180); // rad/s
    private static final double EXIT_HEIGHT = 14;            // inches, where its shots leave
    private static final double HOOD = Math.toRadians(60);
    private static final int PRELOAD = 4;
    private static final double SHOT_EVERY = 0.6;            // s

    // Red's script: shoot from this far out in front of the up cell's opening, across the field from the wall; start
    // against the wall level with that spot; park overlapping the loading zone
    private static final double SHOOT_OUT = 34, SHOOT_X = 40;
    private static final Pose PARK = new Pose(14, 110, Math.PI);
    private final Pose shootFrom;

    final SimMatch.Robot robot;
    private final Alliance alliance;
    private final GamePieceTracker field;
    private double x, y, heading; // Pedro
    private int stage;            // 0 drive to SHOOT, 1 shoot, 2 drive to PARK, 3 parked
    private int shotsLeft = PRELOAD;
    private double wait;

    SimPartner(Alliance alliance, GamePieceTracker field) {
        this.alliance = alliance;
        this.field = field;
        this.robot = new SimMatch.Robot(alliance);
        // In red's view: the rear cell (LEFT) opens toward +y, the audience cell toward -y
        String tray = alliance == Alliance.BLUE ? SimHiveTrays.BLUE : SimHiveTrays.RED;
        boolean rearUp = (field.getBodyAngleRad(tray) > 0) != (alliance == Alliance.BLUE);
        Pose cell = rearUp ? RobotConstants.Field.RED_HIVE_LEFT_CELL : RobotConstants.Field.RED_HIVE_RIGHT_CELL;
        double shootY = cell.y() + (rearUp ? SHOOT_OUT : -SHOOT_OUT);
        shootFrom = new Pose(SHOOT_X, shootY, 0);
        Pose start = alliance.apply(new Pose(9, shootY, 0));
        x = start.x();
        y = start.y();
        heading = start.heading();
        report();
    }

    /** One tick, before the field advances. */
    void update(double deltaTime) {
        switch (stage) {
            case 0:
                if (driveTo(alliance.apply(shootFrom), deltaTime)) {
                    stage = 1;
                    wait = 0.3;
                }
                break;
            case 1:
                wait -= deltaTime;
                if (wait <= 0 && shotsLeft > 0) {
                    shoot();
                    shotsLeft--;
                    wait = SHOT_EVERY;
                }
                if (shotsLeft == 0 && wait <= 0) stage = 2;
                break;
            case 2:
                if (driveTo(alliance.apply(PARK), deltaTime)) stage = 3;
                break;
            default:
                break;
        }
        report();
    }

    /** Straight line at SPEED, turning toward the target's heading on the way. True once there. */
    private boolean driveTo(Pose target, double dt) {
        double dx = target.x() - x, dy = target.y() - y, d = Math.hypot(dx, dy);
        double step = Math.min(d, SPEED * dt);
        if (d > 1e-6) {
            x += dx / d * step;
            y += dy / d * step;
        }
        double turn = Math.IEEEremainder(target.heading() - heading, 2 * Math.PI);
        heading += Math.max(-TURN_RATE * dt, Math.min(TURN_RATE * dt, turn));
        return d - step < 0.01 && Math.abs(turn) < 0.01;
    }

    /** One pollen at the alliance's up cell, from above the partner's center, at the speed that goes in. */
    private void shoot() {
        double fx = FieldFrame.x(x, y), fy = FieldFrame.y(x, y);
        double[] aim = SimHiveTrays.upOpening(field, alliance == Alliance.BLUE);
        double yaw = Math.atan2(aim[1] - fy, aim[0] - fx);
        double speed = SimLauncher.speedToReach(fx, fy, EXIT_HEIGHT, yaw, HOOD, aim);
        double r = Ball.Type.POLLEN.diameter / 2;
        int id = field.addHeldPiece(Ball.Type.POLLEN.name(), r);
        field.launch(id, "partner", fx, fy, EXIT_HEIGHT, speed * Math.cos(HOOD) * Math.cos(yaw),
                speed * Math.cos(HOOD) * Math.sin(yaw), speed * Math.sin(HOOD));
    }

    private void report() {
        robot.x = FieldFrame.x(x, y);
        robot.y = FieldFrame.y(x, y);
        robot.heading = FieldFrame.heading(heading);
        field.setStaticRobots(Collections.singletonList(
                new StaticRobot("partner", robot.x, robot.y, robot.heading, SIZE, SIZE, HEIGHT)));
    }

    void record() {
        Logger.recordOutput("Sim/partner/pose2d", Pose2d.struct, FieldFrame.pose2d(x, y, heading));
    }
}
