package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.FLYWHEEL_MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.SHOT_TABLE_EXTRAPOLATE;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.Launcher.SHOT_TABLE_MERGE;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Flywheel speed by distance, measured on the real robot: each entry is a spot where standing shots went through
 * the middle of the opening at that flywheel speed. Speeds between entries are interpolated in a straight line.
 * Filled by the Shot Table Calibration OpMode.
 *
 * <p>Where the entries come from: /sdcard/FIRST/data/shot_table.csv on the robot (written by the calibration
 * OpMode, so it survives restarts and redeploys), or, if the robot has no such file, the repo's copy,
 * TeamCode/src/main/assets/shot_table.csv, built into the app.
 *
 * <p>Getting the robot's table into the repo: copy the robot's file over the assets file and commit it. Over USB with
 * the Android SDK's adb ({@code adb shell cat /sdcard/FIRST/data/shot_table.csv}), or on the robot's Wi-Fi from
 * http://192.168.43.1:8080/shottable.csv ({@link org.firstinspires.ftc.teamcode.util.ShotTablePage}).
 *
 * <p>Distance is horizontal inches from the turret pivot to the center of the hive cell's opening, as
 * {@code Launcher.distanceTo} measures it. Static, so the calibration OpMode's changes reach the Launcher at once.
 */
public final class ShotTable {
    /** One measured spot. The pose, alliance and cell are kept for the record; only distance and velocity are used. */
    public static class Entry {
        public final double distance; // in, turret pivot to the opening's center
        public final double velocity; // flywheel ticks/sec
        public final double x, y, heading; // robot pose when saved (inches, radians)
        public final String alliance;
        public final String hive;

        public Entry(double distance, double velocity, double x, double y, double heading, String alliance,
                     String hive) {
            this.distance = distance;
            this.velocity = velocity;
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.alliance = alliance;
            this.hive = hive;
        }

        public Entry(double distance, double velocity) {
            this(distance, velocity, Double.NaN, Double.NaN, Double.NaN, "", "");
        }

        public Pose pose() {
            return new Pose(x, y, heading);
        }
    }

    /** The file's name, on the robot and in the repo's assets. */
    public static final String FILE_NAME = "shot_table.csv";
    private static final String HEADER = "distance_in,velocity_ticks_per_s,x,y,heading_deg,alliance,hive";

    private static List<Entry> entries = null; // sorted by distance; null until loaded
    private static String source = "";

    private ShotTable() {}

    // ---- Lookup ----

    /** True if there's at least one entry. */
    public static boolean isEmpty() {
        return entries().isEmpty();
    }

    /** True if the distance is inside the table, or no more than SHOT_TABLE_EXTRAPOLATE past either end. */
    public static boolean covers(double distance) {
        return covers(entries(), distance);
    }

    /**
     * Flywheel ticks/sec for this distance: interpolated between the entries either side, extrapolated from the end
     * two past either end (one entry: that speed everywhere), capped at FLYWHEEL_MAX_VELOCITY. NaN if empty.
     */
    public static double velocityAt(double distance) {
        return velocityAt(entries(), distance);
    }

    static boolean covers(List<Entry> table, double distance) {
        if (table.isEmpty()) return false;
        return distance >= table.get(0).distance - SHOT_TABLE_EXTRAPOLATE
                && distance <= table.get(table.size() - 1).distance + SHOT_TABLE_EXTRAPOLATE;
    }

    static double velocityAt(List<Entry> table, double distance) {
        List<double[]> curve = curve(table);
        int n = curve.size();
        if (n == 0) return Double.NaN;
        if (n == 1) return Math.min(curve.get(0)[1], FLYWHEEL_MAX_VELOCITY);
        // The segment that holds this distance, or the end segment past either end
        int i = 1;
        while (i < n - 1 && distance > curve.get(i)[0]) i++;
        double[] a = curve.get(i - 1), b = curve.get(i);
        double span = b[0] - a[0];
        double velocity = span < 1e-6 ? (a[1] + b[1]) / 2 : a[1] + (distance - a[0]) / span * (b[1] - a[1]);
        return Math.max(0, Math.min(velocity, FLYWHEEL_MAX_VELOCITY));
    }

    /**
     * The points the lookup interpolates between, {distance, ticks/sec}, nearest first: entries within
     * SHOT_TABLE_MERGE of the first in a run are averaged into one point. Several saves at about the same distance
     * so average out each other's noise, instead of making the speed zig-zag between them.
     */
    public static List<double[]> getCurve() {
        return curve(entries());
    }

    static List<double[]> curve(List<Entry> table) {
        List<double[]> points = new ArrayList<>();
        int i = 0;
        while (i < table.size()) {
            double start = table.get(i).distance;
            double distanceSum = 0, velocitySum = 0;
            int count = 0;
            while (i < table.size() && table.get(i).distance - start <= SHOT_TABLE_MERGE) {
                distanceSum += table.get(i).distance;
                velocitySum += table.get(i).velocity;
                count++;
                i++;
            }
            points.add(new double[] {distanceSum / count, velocitySum / count, count});
        }
        return points;
    }

    // ---- Editing (calibration) ----

    /** A copy of the entries, nearest first. */
    public static List<Entry> getEntries() {
        return new ArrayList<>(entries());
    }

    /** Where the entries were loaded from, for telemetry. */
    public static String getSource() {
        entries();
        return source;
    }

    /** Add an entry (every save is kept, however close to another) and write the file. */
    public static void add(Entry entry) {
        addTo(entries(), entry);
        save();
    }

    /** Insert keeping the list sorted by distance; an equal distance goes after the ones already there. */
    static void addTo(List<Entry> table, Entry entry) {
        int at = 0;
        while (at < table.size() && table.get(at).distance <= entry.distance) at++;
        table.add(at, entry);
    }

    /** Remove this exact entry (e.g. the last one saved) and write the file. Returns false if it's not in the table. */
    public static boolean remove(Entry entry) {
        boolean removed = entries().remove(entry);
        if (removed) save();
        return removed;
    }

    /** Remove the entry nearest this distance and write the file. Returns it, or null if the table is empty. */
    public static Entry removeNearest(double distance) {
        List<Entry> table = entries();
        if (table.isEmpty()) return null;
        int best = 0;
        for (int i = 1; i < table.size(); i++) {
            if (Math.abs(table.get(i).distance - distance) < Math.abs(table.get(best).distance - distance)) best = i;
        }
        Entry removed = table.remove(best);
        save();
        return removed;
    }

    /** Remove every entry and write the (empty) file, so SHOT_TABLE isn't used either until the next save. */
    public static void clear() {
        entries().clear();
        save();
    }

    /**
     * Delete the robot's file and go back to the repo's copy built into the app (empty if there is none).
     * For when the repo has a newer table than the robot, e.g. tuned on another Control Hub.
     */
    public static void revertToRepoCopy() {
        try {
            File file = file();
            if (file.isFile() && !file.delete()) throw new IOException("can't delete " + file);
        } catch (IOException | RuntimeException | LinkageError e) {
            RobotLog.ee("ShotTable", e, "Can't delete the shot table file");
        }
        reload();
    }

    // ---- Storage ----

    /**
     * The file. A method rather than a constant: AppUtil loads a native library only the robot has, so on a laptop
     * (unit tests) touching it throws.
     */
    public static File file() {
        return new File(AppUtil.ROBOT_DATA_DIR, FILE_NAME);
    }

    /** Re-read the file (or SHOT_TABLE). */
    public static void reload() {
        entries = null;
        entries();
    }

    private static List<Entry> entries() {
        if (entries != null) return entries;
        List<Entry> loaded = new ArrayList<>();
        source = "empty";
        try {
            File file = file();
            if (file.isFile()) {
                read(new FileReader(file), loaded);
                source = "robot file (" + loaded.size() + ")";
            } else {
                // No table on the robot yet: start from the repo's. The first save writes it all to the robot's file.
                try {
                    read(new InputStreamReader(AppUtil.getDefContext().getAssets().open(FILE_NAME)), loaded);
                    source = "repo copy (" + loaded.size() + ")";
                } catch (FileNotFoundException e) {
                    source = "empty (no robot file or repo copy)";
                }
            }
        } catch (IOException | RuntimeException | LinkageError e) { // LinkageError: no AppUtil off the robot
            RobotLog.ee("ShotTable", e, "Can't read the shot table");
            loaded.clear();
            source = "unreadable: " + e.getMessage();
        }
        entries = loaded;
        return entries;
    }

    private static void read(Reader reader, List<Entry> into) throws IOException {
        try (BufferedReader in = new BufferedReader(reader)) {
            String line;
            while ((line = in.readLine()) != null) {
                Entry entry = parse(line);
                if (entry != null) addTo(into, entry);
            }
        }
    }

    static Entry parse(String line) {
        String[] parts = line.trim().split(",", -1);
        if (parts.length < 2) return null;
        try {
            double distance = Double.parseDouble(parts[0]);
            double velocity = Double.parseDouble(parts[1]);
            double x = parts.length > 2 ? parseOrNaN(parts[2]) : Double.NaN;
            double y = parts.length > 3 ? parseOrNaN(parts[3]) : Double.NaN;
            double heading = parts.length > 4 ? Math.toRadians(parseOrNaN(parts[4])) : Double.NaN;
            String alliance = parts.length > 5 ? parts[5] : "";
            String hive = parts.length > 6 ? parts[6] : "";
            return new Entry(distance, velocity, x, y, heading, alliance, hive);
        } catch (NumberFormatException e) {
            return null; // the header, or a hand-edit gone wrong
        }
    }

    static String format(Entry e) {
        return String.format(Locale.US, "%.2f,%.1f,%.2f,%.2f,%.1f,%s,%s", e.distance, e.velocity, e.x, e.y,
                Math.toDegrees(e.heading), e.alliance, e.hive);
    }

    private static double parseOrNaN(String s) {
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            return Double.NaN;
        }
    }

    private static void save() {
        try {
            File file = file();
            File dir = file.getParentFile();
            if (dir != null && !dir.isDirectory() && !dir.mkdirs()) throw new IOException("can't create " + dir);
            // Write a temporary file and rename it over the old one, so the web page never serves half a table
            File temp = new File(dir, FILE_NAME + ".tmp");
            try (BufferedWriter out = new BufferedWriter(new FileWriter(temp))) {
                out.write(HEADER);
                out.write('\n');
                for (Entry e : entries()) {
                    out.write(format(e));
                    out.write('\n');
                }
            }
            if (!temp.renameTo(file)) throw new IOException("can't replace " + file);
            source = "robot file (" + entries().size() + ")";
        } catch (IOException | RuntimeException | LinkageError e) {
            RobotLog.ee("ShotTable", e, "Can't write the shot table file");
            source = "NOT SAVED to " + FILE_NAME + ": " + e.getMessage();
        }
    }

    /** For tests: use these entries instead of the file. */
    static void setForTest(List<Entry> table) {
        entries = new ArrayList<>();
        for (Entry e : table) addTo(entries, e);
        source = "test";
    }
}
