package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.horizon36596.simloop.field.ConvexShape;
import org.horizon36596.simloop.field.FieldWorld;
import org.horizon36596.simloop.field.GamePiece;
import org.horizon36596.simloop.field.GamePiecePhysicsConfig;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.KinematicBody;
import org.horizon36596.simloop.field.ObstacleBox;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Translation2d;

import java.util.ArrayList;
import java.util.List;

/**
 * The BIOBUZZ field for SimLoop's game pieces: the perimeter, both hive trays (SimHiveTrays), the four flowers, the
 * hive frame's foot bars, the balls' masses and air drag; where the balls start; and the taped zones.
 *
 * Frame: FieldFrame's, inches (x toward the audience, y away from the red wall, z up). Dimensions from the Competition
 * Manual as BiobuzzSim (the Roblox sim) has them, and the flowers from RobotConstants.Field.
 */
final class SimField {
    // ---- Flowers: a tube of stacked pollen, open underneath so the bottom one can be taken ----
    private static final double FLOWER_INNER_RADIUS = 2.25;
    private static final double FLOWER_WALL = 0.4;
    private static final double FLOWER_TUBE_BOTTOM = 3.95; // the gap under it passes pollen (2.8) but not nectar (3.6)
    private static final double FLOWER_TOP = 21.5;
    private static final int POLLEN_PER_FLOWER = 4, POLLEN_PER_GARDEN = 4, NECTAR_PER_CELL = 3;

    // ---- The hive frame's foot bars ----
    static final double FRAME_HALF_WIDTH = 49.46 / 2, FRAME_HALF_DEPTH = 38.95 / 2;

    // ---- Air: the same drag ShotSolver assumes (½ ρ Cd A / m, per inch). No lift, in SimLoop or ShotSolver. ----
    private static final double DRAG_PER_INCH;
    static {
        double r = RobotConstants.Launcher.BALL_DIAMETER / 2 * FieldFrame.METERS_PER_INCH;
        DRAG_PER_INCH = 0.5 * RobotConstants.Launcher.AIR_DENSITY * RobotConstants.Launcher.DRAG_COEFFICIENT
                * Math.PI * r * r / RobotConstants.Launcher.BALL_MASS * FieldFrame.METERS_PER_INCH;
    }

    /** A taped rectangle on the field (or outside it, for the alliance areas), field frame inches. */
    static final class Zone {
        final String name;
        final double x0, x1, y0, y1;

        Zone(String name, double x0, double x1, double y0, double y1) {
            this.name = name;
            this.x0 = x0;
            this.x1 = x1;
            this.y0 = y0;
            this.y1 = y1;
        }

        boolean contains(double x, double y) {
            return x >= x0 && x <= x1 && y >= y0 && y <= y1;
        }

        /** Whether a circle of radius r overlaps it. */
        boolean overlaps(double x, double y, double r) {
            double dx = Math.max(0, Math.max(x0 - x, x - x1)), dy = Math.max(0, Math.max(y0 - y, y - y1));
            return dx * dx + dy * dy < r * r;
        }

        /** Its outline as a closed loop in meters, for an AdvantageScope trajectory. */
        Translation2d[] outline() {
            double m = FieldFrame.METERS_PER_INCH;
            return new Translation2d[] {new Translation2d(x0 * m, y0 * m), new Translation2d(x1 * m, y0 * m),
                    new Translation2d(x1 * m, y1 * m), new Translation2d(x0 * m, y1 * m), new Translation2d(x0 * m, y0 * m)};
        }
    }

    // Loading zones: 24 in along the wall (a tile's seams), 11 in deep. Red's is on its own wall toward the rear (tile
    // A5), blue's toward the audience (F2). Gardens: 23 x 2 in strips along a side wall in the alliance's right-hand
    // corner. Alliance areas: 97 x 54 in outside each alliance wall.
    static final Zone RED_LOADING = new Zone("redLoadingZone", -48, -24, -72, -61);
    static final Zone BLUE_LOADING = new Zone("blueLoadingZone", 24, 48, 61, 72);
    static final Zone RED_GARDEN = new Zone("redGarden", 70, 72, -71.5, -48.5);
    static final Zone BLUE_GARDEN = new Zone("blueGarden", -72, -70, 48.5, 71.5);
    static final Zone RED_ALLIANCE_AREA = new Zone("redAllianceArea", -48.5, 48.5, -127, -73);
    static final Zone BLUE_ALLIANCE_AREA = new Zone("blueAllianceArea", -48.5, 48.5, 73, 127);
    static final Zone[] ZONES = {RED_LOADING, BLUE_LOADING, RED_GARDEN, BLUE_GARDEN, RED_ALLIANCE_AREA, BLUE_ALLIANCE_AREA};

    /** Air drag on a ball, per inch: the k in a = -k |v| v. */
    static double dragPerInch() {
        return DRAG_PER_INCH;
    }

    static Zone loadingZone(Alliance alliance) {
        return alliance == Alliance.RED ? RED_LOADING : BLUE_LOADING;
    }

    static Zone garden(Alliance alliance) {
        return alliance == Alliance.RED ? RED_GARDEN : BLUE_GARDEN;
    }

    private SimField() {}

    /** The world: set each tray's starting angle (SimHiveTrays.angle) before staging or the first advance. */
    static FieldWorld world() {
        GamePiecePhysicsConfig physics = GamePiecePhysicsConfig.builder()
                .pieceMass(Ball.Type.POLLEN.name(), SimHiveTrays.POLLEN_MASS)
                .pieceMass(Ball.Type.RED_NECTAR.name(), SimHiveTrays.NECTAR_MASS)
                .pieceMass(Ball.Type.BLUE_NECTAR.name(), SimHiveTrays.NECTAR_MASS)
                .airDragPerInch(DRAG_PER_INCH)
                .build();
        // The robot pushes balls it drives into (it's a box to them: SimLoop doesn't push the robot back)
        FieldWorld.Builder world = FieldWorld.builder().physics(physics).perimeter(72, 72)
                .robotFootprint(RobotConstants.Field.ROBOT_LENGTH, RobotConstants.Field.ROBOT_WIDTH)
                .kinematicBody(SimHiveTrays.tray(SimHiveTrays.RED, false))
                .kinematicBody(SimHiveTrays.tray(SimHiveTrays.BLUE, true));
        for (int side = -1; side <= 1; side += 2) {
            double y = side * FRAME_HALF_WIDTH;
            world.obstacle(new ObstacleBox("hiveFoot" + (side < 0 ? "Red" : "Blue"),
                    -FRAME_HALF_DEPTH - 1, FRAME_HALF_DEPTH + 1, y - 1, y + 1, 0, 1.5));
        }
        for (int i = 0; i < RobotConstants.Field.FLOWERS.length; i++) {
            world.kinematicBody(flower(i));
        }
        return world.build();
    }

    /**
     * Flower i's column as the chassis meets it: a square around the tube, field frame, from the gap under it up. The
     * intake's mouth reaches past the bumper to under the tube's middle, so the bottom ball can still be taken.
     */
    static ObstacleBox flowerColumn(int i) {
        double[] c = flowerCenter(i);
        double r = FLOWER_INNER_RADIUS + FLOWER_WALL;
        return new ObstacleBox("flower" + i, c[0] - r, c[0] + r, c[1] - r, c[1] + r, FLOWER_TUBE_BOTTOM, FLOWER_TOP);
    }

    /** Field frame (inches) of flower i's axis. */
    static double[] flowerCenter(int i) {
        Pose p = RobotConstants.Field.FLOWERS[i];
        return new double[] {FieldFrame.x(p.x(), p.y()), FieldFrame.y(p.x(), p.y())};
    }

    /** A flower that never moves: the tube from its gap to its top, in field coordinates. */
    private static KinematicBody flower(int i) {
        double[] c = flowerCenter(i);
        List<ConvexShape> shapes = new ArrayList<>(SimShapes.ring("flower" + i, c[0], c[1], FLOWER_INNER_RADIUS,
                FLOWER_WALL, FLOWER_TUBE_BOTTOM, FLOWER_TOP, 12));
        return new KinematicBody("flower" + i, new double[] {0, 0, 0}, new double[] {0, 0, 1}, shapes, null, 0.1, 0.5);
    }

    /**
     * Balls scattered over the tiles instead of the match's staging, as after a few tips: pollen, and nectar of both
     * colors split evenly. Seeded, so a seed gives the same field every run. Each ball keeps clear of the flowers (the
     * chase's FLOWER_KEEP_OUT plus a ball), the hive's feet, the walls, the other balls and a robot-sized box around
     * (avoidX, avoidY), in Pedro inches.
     *
     * @return where each went, Pedro inches: {type index (0 pollen, 1 red nectar, 2 blue nectar), x, y}
     */
    static List<double[]> scatter(GamePieceTracker field, long seed, int pollen, int nectar, double avoidX, double avoidY) {
        java.util.Random random = new java.util.Random(seed);
        List<double[]> placed = new ArrayList<>();
        Ball.Type[] types = {Ball.Type.POLLEN, Ball.Type.RED_NECTAR, Ball.Type.BLUE_NECTAR};
        double clear = Math.hypot(RobotConstants.Field.ROBOT_LENGTH, RobotConstants.Field.ROBOT_WIDTH) / 2 + 3;
        for (int n = 0; n < pollen + nectar; n++) {
            int type = n < pollen ? 0 : 1 + (n - pollen) % 2;
            double r = types[type].diameter / 2;
            for (int attempt = 0; attempt < 1000; attempt++) {
                double x = 4 + random.nextDouble() * 136, y = 4 + random.nextDouble() * 136;
                if (Math.hypot(x - avoidX, y - avoidY) < clear || !clearOfField(x, y, r)) continue;
                boolean apart = true;
                for (double[] other : placed) apart &= Math.hypot(x - other[1], y - other[2]) > 3 * r;
                if (!apart) continue;
                field.addLoosePiece(types[type].name(), FieldFrame.x(x, y), FieldFrame.y(x, y), r);
                placed.add(new double[] {type, x, y});
                break;
            }
        }
        return placed;
    }

    /** Off the flowers and the hive's feet (Pedro inches). */
    private static boolean clearOfField(double x, double y, double r) {
        for (Pose flower : RobotConstants.Field.FLOWERS) {
            if (Math.hypot(x - flower.x(), y - flower.y()) < RobotConstants.Field.FLOWER_KEEP_OUT + r) return false;
        }
        double m = RobotConstants.Field.KEEP_OUT_MARGIN + r;
        for (double[] foot : RobotConstants.Field.HIVE_FEET) {
            if (x > foot[0] - m && x < foot[2] + m && y > foot[1] - m && y < foot[3] + m) return false;
        }
        return true;
    }

    /**
     * The balls a match starts with on the field (Competition Manual, as BiobuzzSim stages them): 4 pollen stacked in
     * each flower, 4 in a row in each garden, 3 nectar in each hive's up cell. Not here: the 5 nectar in each alliance
     * area (the human player's) and each robot's preload, which BiobuzzRobotSim gives its robot.
     */
    static void stage(GamePieceTracker field) {
        stage(field, null);
    }

    /**
     * stage(), with each garden's row of 4 placed by hand rather than measured: moved 0 to 6 in toward the center and
     * up to 1.5 in more between balls (seeded; null: as BiobuzzSim lays them).
     */
    static void stage(GamePieceTracker field, java.util.Random jitter) {
        double r = Ball.Type.POLLEN.diameter / 2;
        for (int i = 0; i < RobotConstants.Field.FLOWERS.length; i++) {
            double[] c = flowerCenter(i);
            for (int k = 0; k < POLLEN_PER_FLOWER; k++) {
                int id = field.addPieceAt(Ball.Type.POLLEN.name(), c[0], c[1], r + k * (2 * 0.999 * r), r);
                asleep(field.getPiece(id));
            }
        }
        for (int garden = 0; garden < 2; garden++) {
            double along = 71.5 - 0.6 - r - (jitter == null ? 0 : jitter.nextDouble() * 6);
            for (int k = 0; k < POLLEN_PER_GARDEN; k++) {
                if (garden == 0) field.addLoosePiece(Ball.Type.POLLEN.name(), 72 - 0.5 - r, -along, r);  // red garden
                else field.addLoosePiece(Ball.Type.POLLEN.name(), -72 + 0.5 + r, along, r);             // blue garden
                along -= 2 * r + 0.15 + (jitter == null ? 0 : jitter.nextDouble() * 1.5);
            }
        }
        SimHiveTrays.stageNectar(field, false, NECTAR_PER_CELL);
        SimHiveTrays.stageNectar(field, true, NECTAR_PER_CELL);
    }

    /**
     * Puts a staged piece to sleep, as the physics does with one that has come to rest: it stays exactly where it is
     * until something near it moves or another piece hits it. The flowers' stacks need it: balls stacked straight up
     * are balanced on a point, and with the tube open underneath the bottom one rolled out before the match started
     * (the rest after it), so the robot chased the flowers' pollen off the tiles; on the field it stays in the flower.
     * SimLoop has no public setter, so by reflection (as BiobuzzRobotSim moves the Limelight's tags).
     */
    private static void asleep(GamePiece piece) {
        try {
            java.lang.reflect.Method set = GamePiece.class.getDeclaredMethod("setSleeping", boolean.class);
            set.setAccessible(true);
            set.invoke(piece, true);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("SimLoop's GamePiece.setSleeping changed", e);
        }
    }

    /** How many balls sit in the garden: any part of one over the tape. */
    static int ballsIn(GamePieceTracker field, Zone zone) {
        int n = 0;
        for (GamePiece p : field.getPieces()) {
            if (p.isVisibleOnField() && p.getFieldZInches() < 2 * p.getRadiusInches()
                    && zone.overlaps(p.getFieldXInches(), p.getFieldYInches(), p.getRadiusInches())) n++;
        }
        return n;
    }

    /** The zones as AdvantageScope trajectories (Field/zones/...), drawn on the floor of the 3D field. */
    static void recordZones() {
        for (Zone zone : ZONES) {
            Logger.recordOutput("Field/zones/" + zone.name, Translation2d.struct, zone.outline());
        }
    }
}
