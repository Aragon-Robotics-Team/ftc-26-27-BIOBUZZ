package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.horizon36596.simloop.field.BodyCell;
import org.horizon36596.simloop.field.ConvexShape;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.HingeDynamics;
import org.horizon36596.simloop.field.KinematicBody;

import java.util.ArrayList;
import java.util.List;

/**
 * The hive trays as SimLoop bodies that tip by themselves: each tray is a {@link KinematicBody} on a hinge
 * ({@link HingeDynamics}), turned by its own weight and by the balls in its cells, between stops near ±30° (STOPS:
 * where the shock absorbers let it rest). Nothing here decides when a hive tips; the balls' weight does.
 *
 * Frame: the game pieces' field frame (FieldFrame, inches: x toward the audience, y away from the red wall, z up).
 * Each tray's local frame is that frame moved to its pivot, so at angle 0 the tray is level with its cells along x:
 * the rear cell (red's LEFT, blue's RIGHT) at -x, the audience cell at +x. The hinge axis is +y, so a positive angle
 * raises the rear cell; that angle is the tray pose's pitch (SimHives.trayPose).
 *
 * Shapes (Competition Manual 9.6, as BiobuzzSim has them): each cell is a 20 in wide, 12 in deep box with a peaked
 * roof, open at the end away from the pivot, its floor 1.25 in below the arm. When a cell is up its floor slopes down
 * toward the pivot, so balls stay; when it comes down the floor slopes out and they roll out of the opening.
 *
 * How heavy it is: the Field Setup Guide (12.3) says 8 pollen tip a hive, 3 pollen and 3 nectar tip it, and 7 pollen
 * don't. Pollen weighs 1 here and nectar 5/3, and the tray's mass and centre of mass are set so its own weight holds
 * the up cell against 7 pollen but not 8 (HiveTippingSimTest checks all three). The tray's own mass, inertia and
 * damping are PLACEHOLDERS chosen for that and for a tip of about 2 s; weigh a tray and time a tip to replace them.
 */
final class SimHiveTrays {
    static final String RED = "redHive", BLUE = "blueHive";

    // ---- Shapes, tray-local inches ----
    private static final double ARM_HALF = 18.84 / 2;  // pivot to each cell's back wall
    private static final double CELL_WIDTH = 20, CELL_DEPTH = 12, CELL_HEIGHT = 14, CELL_SIDE_HEIGHT = 7.61;
    private static final double FLOOR_BELOW_ARM = 1.25; // floor's underside below the arm axis
    private static final double SKIN = 0.5;            // wall thickness
    // Where each arm comes to rest: on its shock absorbers, a little short of 30°, and not quite the same either way
    // or on both hives, since they don't always bottom out. Red's {rear down, rear up}, then blue's. PLACEHOLDER spread
    private static final double[][] STOPS = {
            {Math.toRadians(-28.6), Math.toRadians(29.4)},
            {Math.toRadians(-29.3), Math.toRadians(28.7)}};

    // ---- Weights: pollen is the unit ----
    static final double POLLEN_MASS = 1, NECTAR_MASS = 5.0 / 3;
    // Tray mass and how far its centre of mass sits above the pivot. Only their product matters for when it tips:
    // its hold at a stop is mass * g * height * sin 30°. PLACEHOLDERS, tuned to the 7-holds / 8-tips rule.
    private static final double TRAY_MASS = 60;
    private static final double TRAY_COM_HEIGHT = 2.5;
    private static final double TRAY_INERTIA = TRAY_MASS * 12 * 12; // radius of gyration 12 in. PLACEHOLDER
    private static final double TRAY_DAMPING = 0.5;                  // 1/s. PLACEHOLDER
    // The cell skins are fabric: balls barely bounce on them
    private static final double TRAY_RESTITUTION = 0.1, TRAY_FRICTION = 0.5;

    private SimHiveTrays() {}

    /** The tray's angle at rest with the given cell up, as that alliance sees it: positive raises the rear cell. */
    static double angle(boolean blue, Hive up) {
        boolean rearUp = (up == Hive.LEFT) != blue; // red's LEFT is the rear cell, blue's LEFT the audience one
        return STOPS[blue ? 1 : 0][rearUp ? 1 : 0];
    }

    /** Which cell is up at this angle, as that alliance sees it; null while the tray is between the stops. */
    static Hive up(boolean blue, double angle) {
        boolean rearUp = angle > 0;
        if (Math.abs(angle - STOPS[blue ? 1 : 0][rearUp ? 1 : 0]) > Math.toRadians(1)) return null;
        return rearUp != blue ? Hive.LEFT : Hive.RIGHT;
    }

    /** The cell's name in the tracker: e.g. "redHive_rear" or "blueHive_audience". */
    static String cellName(String tray, boolean rear) {
        return tray + (rear ? "_rear" : "_audience");
    }

    /**
     * Sets a ball down gently in a tray's up cell, as if it had just landed there, and returns its id. The slot spreads
     * balls out: six across a row, each row 3 in further from the back wall. Balls roll to the back wall and settle.
     */
    static int dropIntoUpCell(GamePieceTracker field, boolean blue, Ball.Type type, int slot) {
        String tray = blue ? BLUE : RED;
        double angle = field.getBodyAngleRad(tray);
        double s = angle > 0 ? -1 : 1; // the up cell: rear (-x) when the angle is positive
        double r = type.diameter / 2;
        double along = ARM_HALF + SKIN + 2 + r + (slot / 6) * 3.0;
        double[] at = new double[3];
        field.getWorld().getKinematicBody(tray).toField(angle, s * along, ((slot % 6) - 2.5) * 3.0,
                -FLOOR_BELOW_ARM + SKIN + r + 0.2, at);
        return field.addPieceAt(type.name(), at[0], at[1], at[2], r);
    }

    /**
     * The match's starting nectar: three in each hive's up cell, along its back wall from the side nearest that
     * alliance's wall (BiobuzzSim's staging). Five nectar are not enough to tip it.
     */
    static void stageNectar(GamePieceTracker field, boolean blue, int count) {
        String tray = blue ? BLUE : RED;
        double angle = field.getBodyAngleRad(tray);
        double s = angle > 0 ? -1 : 1;
        double r = Ball.Type.RED_NECTAR.diameter / 2;
        double ownWall = blue ? 1 : -1; // the alliance's own wall is toward -y for red
        Ball.Type type = blue ? Ball.Type.BLUE_NECTAR : Ball.Type.RED_NECTAR;
        for (int i = 0; i < count; i++) {
            double across = ownWall * (CELL_WIDTH / 2 - SKIN - r - 0.1 - i * (2 * r + 0.2));
            double[] at = new double[3];
            field.getWorld().getKinematicBody(tray).toField(angle, s * (ARM_HALF + SKIN + r + 0.2), across,
                    -FLOOR_BELOW_ARM + SKIN + r + 0.3, at);
            field.addPieceAt(type.name(), at[0], at[1], at[2], r);
        }
    }

    /** The middle of the up cell's opening, field frame (inches): what a shot aims at. */
    static double[] upOpening(GamePieceTracker field, boolean blue) {
        String tray = blue ? BLUE : RED;
        double angle = field.getBodyAngleRad(tray);
        double s = angle > 0 ? -1 : 1;
        double[] at = new double[3];
        field.getWorld().getKinematicBody(tray).toField(angle, s * (ARM_HALF + CELL_DEPTH), 0,
                -FLOOR_BELOW_ARM + CELL_HEIGHT / 2, at);
        return at;
    }

    static KinematicBody tray(String name, boolean blue) {
        double pivotY = (blue ? 1 : -1) * SimHives.HIVE_FROM_CENTER;
        List<ConvexShape> shapes = new ArrayList<>();
        List<BodyCell> cells = new ArrayList<>();
        shapes.add(ConvexShape.box(name + "_arm", -ARM_HALF - 1, ARM_HALF + 1, -1, 1, -1, 1));
        for (boolean rear : new boolean[] {true, false}) {
            addCell(shapes, cells, cellName(name, rear), rear ? -1 : 1);
        }
        KinematicBody body = new KinematicBody(name, new double[] {0, pivotY, SimHives.PIVOT_HEIGHT},
                new double[] {0, 1, 0}, shapes, cells, TRAY_RESTITUTION, TRAY_FRICTION);
        return body.withHinge(new HingeDynamics(TRAY_MASS, new double[] {0, 0, TRAY_COM_HEIGHT}, TRAY_INERTIA,
                TRAY_DAMPING, STOPS[blue ? 1 : 0][0], STOPS[blue ? 1 : 0][1], 0));
    }

    /** One cell, on the side s (-1 rear, +1 audience): floor, two sides, the back wall with its peak, the roof. */
    private static void addCell(List<ConvexShape> shapes, List<BodyCell> cells, String name, double s) {
        double back = ARM_HALF, open = ARM_HALF + CELL_DEPTH; // distances from the pivot along the arm
        double w = CELL_WIDTH / 2;
        double floor = -FLOOR_BELOW_ARM, eaves = floor + CELL_SIDE_HEIGHT, peak = floor + CELL_HEIGHT;
        // Every part is a prism along x: a convex outline across the cell (y, z), from one distance to another
        shapes.add(prism(name + "_floor", s, back, open, new double[][] {{-w, floor}, {w, floor}, {w, floor + SKIN},
                {-w, floor + SKIN}}));
        for (int side = -1; side <= 1; side += 2) {
            double o = side * w, i = side * (w - SKIN);
            shapes.add(prism(name + (side < 0 ? "_right" : "_left"), s, back, open,
                    new double[][] {{Math.min(o, i), floor}, {Math.max(o, i), floor}, {Math.max(o, i), eaves},
                            {Math.min(o, i), eaves}}));
            shapes.add(prism(name + (side < 0 ? "_roofRight" : "_roofLeft"), s, back, open,
                    new double[][] {{o, eaves - SKIN}, {o, eaves}, {0, peak}, {0, peak - SKIN}}));
        }
        shapes.add(prism(name + "_back", s, back, back + SKIN, new double[][] {{-w, floor}, {w, floor}, {w, eaves},
                {0, peak}, {-w, eaves}}));
        double near = back + SKIN, far = open;
        cells.add(new BodyCell(name, s < 0 ? -far : near, s < 0 ? -near : far, -(w - SKIN), w - SKIN,
                floor + SKIN, peak - SKIN, 1.0));
    }

    /** A prism along x on the side s, from x0 to x1 out from the pivot, over a convex (y, z) outline. */
    private static ConvexShape prism(String name, double s, double x0, double x1, double[][] outline) {
        return SimShapes.alongX(name, s * x0, s * x1, outline);
    }
}
