package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.psilynx.psikit.core.Logger;

import java.util.List;

/**
 * The match clock and the score, as the Competition Manual counts it (point values as BiobuzzSim has them): 20 per
 * tip, 2 per ball in the alliance's up cell, 1 per ball in its garden, 3 for leaving the wall in AUTO, 5 for parking in
 * the loading zone (at the end of AUTO, and again at the end of TELEOP). Logged every tick under Match/ as if the
 * match ended then, so a line graph shows the score building.
 *
 * Not counted: flowers (the sim has no nectar placement), fouls, and ranking points. One OpMode run is one period: an
 * Auto run is the 30 s AUTO, anything else the 2:00 TELEOP; the clock starts when the OpMode does.
 */
final class SimMatch {
    static final double AUTO_SECONDS = 30, TELEOP_SECONDS = 120;
    private static final int TIP = 20, CELL_BALL = 2, GARDEN_BALL = 1, LEAVE = 3, PARK = 5;
    private static final double WALL_TOUCH = 0.5; // inches: closer than this to a wall counts as touching it

    /** One robot on the field: its alliance and where its frame is (field frame, inches, radians). */
    static final class Robot {
        final Alliance alliance;
        double x, y, heading;
        boolean leftWall;

        Robot(Alliance alliance) {
            this.alliance = alliance;
        }
    }

    private final GamePieceTracker field;
    private final List<Robot> robots;
    private boolean auto;
    private boolean running;
    private double time;
    private final int[] tips = new int[2];
    private final double[] lastStop = new double[2];

    SimMatch(GamePieceTracker field, List<Robot> robots) {
        this.field = field;
        this.robots = robots;
        lastStop[0] = Math.signum(field.getBodyAngleRad(SimHiveTrays.RED));
        lastStop[1] = Math.signum(field.getBodyAngleRad(SimHiveTrays.BLUE));
    }

    /** Which period this run is. Set before the OpMode starts. */
    void setAuto(boolean auto) {
        this.auto = auto;
    }

    /** The OpMode has started: the clock runs from here. */
    void start() {
        running = true;
    }

    double timeLeft() {
        return Math.max(0, (auto ? AUTO_SECONDS : TELEOP_SECONDS) - time);
    }

    int tips(Alliance alliance) {
        return tips[alliance.ordinal()];
    }

    int score(Alliance alliance) {
        return tips(alliance) * TIP + cellBalls(alliance) * CELL_BALL
                + SimField.ballsIn(field, SimField.garden(alliance)) * GARDEN_BALL
                + (auto ? count(alliance, true) * LEAVE : 0) + count(alliance, false) * PARK;
    }

    /** After the field has advanced this tick. */
    void update(double deltaTime) {
        if (!running) return;
        if (timeLeft() > 0) time += deltaTime;
        for (int i = 0; i < 2; i++) {
            double angle = field.getBodyAngleRad(i == 0 ? SimHiveTrays.RED : SimHiveTrays.BLUE);
            boolean atStop = SimHiveTrays.up(i == 1, angle) != null;
            if (atStop && Math.signum(angle) != lastStop[i] && timeLeft() > 0) {
                tips[i]++;
                lastStop[i] = Math.signum(angle);
            }
        }
        for (Robot r : robots) {
            if (!touchingWall(r)) r.leftWall = true;
        }
    }

    /** Balls in the alliance's up cell: the one on the side of the tray that's raised. */
    /** The alliance's robots parked in its loading zone now. */
    int parked(Alliance alliance) {
        return count(alliance, false);
    }

    int cellBalls(Alliance alliance) {
        String tray = alliance == Alliance.RED ? SimHiveTrays.RED : SimHiveTrays.BLUE;
        boolean rearUp = field.getBodyAngleRad(tray) > 0;
        return field.getScoredCountIn(SimHiveTrays.cellName(tray, rearUp));
    }

    /** Robots of the alliance that have left the wall (leave = true) or are parked in its loading zone. */
    private int count(Alliance alliance, boolean leave) {
        int n = 0;
        for (Robot r : robots) {
            if (r.alliance != alliance) continue;
            if (leave ? r.leftWall : parked(r)) n++;
        }
        return n;
    }

    private static double[][] corners(Robot r) {
        double hl = RobotConstants.Field.ROBOT_LENGTH / 2, hw = RobotConstants.Field.ROBOT_WIDTH / 2;
        double c = Math.cos(r.heading), s = Math.sin(r.heading);
        double[][] out = new double[4][];
        int k = 0;
        for (int i = -1; i <= 1; i += 2) {
            for (int j = -1; j <= 1; j += 2) {
                out[k++] = new double[] {r.x + i * hl * c - j * hw * s, r.y + i * hl * s + j * hw * c};
            }
        }
        return out;
    }

    private static boolean touchingWall(Robot r) {
        for (double[] p : corners(r)) {
            if (72 - Math.abs(p[0]) <= WALL_TOUCH || 72 - Math.abs(p[1]) <= WALL_TOUCH) return true;
        }
        return false;
    }

    /** Any part of the robot over its loading zone: a corner, the center, or the middle of a side. */
    private static boolean parked(Robot r) {
        SimField.Zone zone = SimField.loadingZone(r.alliance);
        double[][] c = corners(r);
        if (zone.contains(r.x, r.y)) return true;
        int[] around = {0, 1, 3, 2}; // corners() lists them (-,-) (-,+) (+,-) (+,+)
        for (int i = 0; i < 4; i++) {
            double[] a = c[around[i]], b = c[around[(i + 1) % 4]];
            if (zone.contains(a[0], a[1]) || zone.contains((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)) return true;
        }
        return false;
    }

    void record() {
        Logger.recordOutput("Match/period", !running ? "PRE-MATCH" : timeLeft() <= 0 ? "ENDED" : auto ? "AUTO" : "TELEOP");
        Logger.recordOutput("Match/timeLeft", timeLeft());
        for (Alliance a : Alliance.values()) {
            String k = "Match/" + a.name().toLowerCase() + "/";
            Logger.recordOutput(k + "tips", tips(a));
            Logger.recordOutput(k + "cellBalls", cellBalls(a));
            Logger.recordOutput(k + "gardenBalls", SimField.ballsIn(field, SimField.garden(a)));
            Logger.recordOutput(k + "left", count(a, true));
            Logger.recordOutput(k + "parked", count(a, false));
            Logger.recordOutput(k + "score", score(a));
        }
    }
}
