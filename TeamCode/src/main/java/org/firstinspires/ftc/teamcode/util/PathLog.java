package org.firstinspires.ftc.teamcode.util;

import android.media.MediaScannerConnection;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerLog;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.routines.PlannedPath;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.function.Consumer;

/**
 * Writes how the follower drove, one CSV row per loop, for the Logs tab of tools/path-planner (it checks the speed
 * model against real runs). One file per run in /sdcard/FIRST/data/pathlogs, never deleted. Download them from
 * http://192.168.43.1:8080/pathlogs (see {@link PathLogsPage}) or over USB.
 * The columns are described in tools/path-planner/src/core/logs.ts; keep both in step.
 */
public class PathLog implements Consumer<FollowerLog> {
    public static final File DIR = new File(AppUtil.ROBOT_DATA_DIR, "pathlogs");
    private static final long VOLTAGE_EVERY_NS = 250_000_000L; // reading the battery takes a hub round trip

    private final Follower follower;
    private final HardwareMap hardwareMap;
    private final File file;
    private final BufferedWriter out;
    private final long startNs = System.nanoTime();
    private long voltageAtNs = Long.MIN_VALUE;
    private long flushedNs = startNs;
    private double voltage = Double.NaN;
    private boolean closed;

    private PathLog(Follower follower, HardwareMap hardwareMap, File file, BufferedWriter out) {
        this.follower = follower;
        this.hardwareMap = hardwareMap;
        this.file = file;
        this.out = out;
    }

    /**
     * Starts logging a run and hooks into the follower so every update writes a row. Returns null (after noting why in
     * the robot log) if the file can't be created: a missing log must never stop an Auto.
     */
    public static PathLog start(HardwareMap hardwareMap, Follower follower, String routine, Alliance alliance) {
        try {
            if (!DIR.isDirectory() && !DIR.mkdirs()) throw new IOException("can't create " + DIR);
            String stamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(new Date());
            File file = new File(DIR, stamp + "_" + routine + "_" + alliance + ".csv");
            BufferedWriter out = new BufferedWriter(new FileWriter(file), 1 << 16);
            out.write("# path-planner log v1\n# routine=" + routine + "\n# alliance=" + alliance + "\n");
            out.write("t,voltage,x,y,heading,mode,busy,path,segment,tparam\n");
            PathLog log = new PathLog(follower, hardwareMap, file, out);
            follower.withLogger(log);
            return log;
        } catch (IOException | RuntimeException e) {
            RobotLog.ee("PathLog", e, "Not logging this run");
            return null;
        }
    }

    @Override
    public void accept(FollowerLog ignored) {
        if (closed) return;
        long now = System.nanoTime();
        if (now - voltageAtNs >= VOLTAGE_EVERY_NS) {
            voltageAtNs = now;
            voltage = batteryVoltage();
        }
        Pose pose = follower.pose();
        String path = follower.following() ? PlannedPath.idOf(follower.currentPath()) : null;
        try {
            out.write(String.format(Locale.US, "%.4f,%.2f,%.3f,%.3f,%.4f,%s,%d,%s,%d,%.4f\n",
                    (now - startNs) / 1e9, voltage, pose.x(), pose.y(), pose.heading(), follower.mode(),
                    follower.isBusy() ? 1 : 0, path == null ? "" : path, follower.pathIndex(),
                    follower.following() ? follower.parametricCompletion() : 0.0));
            // Flush now and then, so a run that ends abruptly still leaves most of its log.
            if (now - flushedNs > 1_000_000_000L) {
                flushedNs = now;
                out.flush();
            }
        } catch (IOException e) {
            RobotLog.ee("PathLog", e, "Stopped logging this run");
            close();
        }
    }

    private double batteryVoltage() {
        double lowest = Double.NaN;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0 && !(v >= lowest)) lowest = v;
        }
        return lowest;
    }

    /** Finishes the file and tells Android about it, so it shows up over USB without a restart. */
    public void close() {
        if (closed) return;
        closed = true;
        try {
            out.close();
        } catch (IOException e) {
            RobotLog.ee("PathLog", e, "Couldn't finish " + file);
        }
        MediaScannerConnection.scanFile(AppUtil.getDefContext(), new String[] {file.getAbsolutePath()}, null, null);
    }
}
