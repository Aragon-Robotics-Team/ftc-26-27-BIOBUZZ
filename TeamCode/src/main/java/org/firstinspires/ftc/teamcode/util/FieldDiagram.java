package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.RobotConstants.Field.FLOWERS;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_HIVE_LEFT_CELL;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_HIVE_RIGHT_CELL;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Hive;

/**
 * Top-down field for telemetry, one character per cell, as seen from our driver station: our wall at the bottom,
 * our left on the left. Each row is one telemetry line; wrap it in {@link Html#mono} so the columns line up.
 * tools/telemetry-preview.html draws the same diagram; keep it in step.
 *
 * <pre>
 *   ########@###############   a flower, on the wall
 *   #########======#########   the other alliance's hive, in their color
 *   #########=====O#########   ours, in our color: O is the up cell (here the right)
 *   #####↗##################   the robot, pointing the way it faces
 * </pre>
 */
public final class FieldDiagram {
    public static final int ROWS = 12; // 12 in per row
    public static final int COLS = 24; // 6 in per column: a monospace character is about half as wide as a line is tall

    public static final String FLOOR = Html.DARK_GRAY;
    public static final String FLOWER = "#FFA000"; // orange

    // Screen directions for headings 0°, 45°, 90°, ... (0° = away from our wall, 90° = our left).
    // If a row with the robot in it looks shifted on the Driver Hub, its monospace font lacks arrows: use ^ < v > here.
    private static final String[] ARROWS = {"↑", "↖", "←", "↙", "↓", "↘", "→", "↗"};

    private static final double FIELD = 144;

    private FieldDiagram() {}

    /**
     * @param robotPose where the robot is (Pedro coordinates)
     * @return ROWS strings of COLS characters each
     */
    public static String[] draw(Pose robotPose, Alliance alliance, Hive hive) {
        String ours = Html.allianceColor(alliance);
        String theirs = Html.allianceColor(alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED);

        String[][] text = new String[ROWS][COLS];
        String[][] color = new String[ROWS][COLS];
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) set(text, color, r, c, "#", FLOOR);
        }

        // Drawn in red-side coordinates, where our driver station is at x = 0 and our left is +y.
        // Alliance.apply turns a blue pose into the matching red one (the field is the same rotated 180°).
        Pose robot = alliance.apply(robotPose);
        Pose up = hive == Hive.LEFT ? RED_HIVE_LEFT_CELL : RED_HIVE_RIGHT_CELL;

        // Hives: a bar from one cell to the other. Ours is on our side of the field; theirs is rotated 180°.
        drawHive(text, color, FIELD - RED_HIVE_LEFT_CELL.x(), FIELD - RED_HIVE_LEFT_CELL.y(), FIELD - RED_HIVE_RIGHT_CELL.y(),
                theirs);
        drawHive(text, color, RED_HIVE_LEFT_CELL.x(), RED_HIVE_LEFT_CELL.y(), RED_HIVE_RIGHT_CELL.y(), ours);
        set(text, color, row(up.x()), col(up.y()), Html.bold("O"), ours);

        for (Pose flower : FLOWERS) set(text, color, row(flower.x()), col(flower.y()), Html.bold("@"), FLOWER);

        int direction = (int) Math.round(Math.toDegrees(robot.heading()) / 45);
        set(text, color, row(robot.x()), col(robot.y()), Html.bold(ARROWS[((direction % 8) + 8) % 8]), ours);

        String[] rows = new String[ROWS];
        for (int r = 0; r < ROWS; r++) rows[r] = runs(text[r], color[r]);
        return rows;
    }

    /** @param y1 / y2 the y of each of the hive's cells */
    private static void drawHive(String[][] text, String[][] color, double x, double y1, double y2, String hiveColor) {
        int r = row(x), from = Math.min(col(y1), col(y2)), to = Math.max(col(y1), col(y2));
        for (int c = from; c <= to; c++) set(text, color, r, c, "=", hiveColor);
    }

    private static void set(String[][] text, String[][] color, int r, int c, String character, String cellColor) {
        text[r][c] = character;
        color[r][c] = cellColor;
    }

    /** One font tag per run of same-colored characters, to keep the telemetry packet small. */
    private static String runs(String[] text, String[] colors) {
        StringBuilder out = new StringBuilder();
        int start = 0;
        for (int c = 1; c <= colors.length; c++) {
            if (c == colors.length || !colors[c].equals(colors[start])) {
                StringBuilder run = new StringBuilder();
                for (int i = start; i < c; i++) run.append(text[i]);
                out.append(Html.color(run.toString(), colors[start]));
                start = c;
            }
        }
        return out.toString();
    }

    // Screen row 0 is the far wall (x = 144); column 0 is our left (y = 144)
    private static int row(double x) {
        return clamp((int) Math.floor((FIELD - x) / FIELD * ROWS), ROWS);
    }

    private static int col(double y) {
        return clamp((int) Math.floor((FIELD - y) / FIELD * COLS), COLS);
    }

    private static int clamp(int i, int n) {
        return Math.max(0, Math.min(n - 1, i));
    }
}
