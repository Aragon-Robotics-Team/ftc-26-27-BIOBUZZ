package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * PsiKit logging for AdvantageScope. An OpMode calls start() in init, beginLoop() and endLoop() around each loop, and
 * stop(); Robot.log() records the robot's state in between.
 *
 * On the robot it all does nothing unless RobotConstants.Logging.ENABLED: then each run is written to
 * FIRST/data/logs/&lt;OpMode&gt;-&lt;time&gt;.rlog. In the sim, SimLoop already runs PsiKit for its own log, so only the
 * robot's state is added to it, whatever the flag says.
 */
public final class MatchLog {
    /** Running under SimLoop on a laptop: its runner owns PsiKit, so this must not start or step it. */
    private static final boolean SIM = classExists("org.horizon36596.simloop.opmode.OpModeRunner");
    private static final double METERS_PER_INCH = 0.0254;
    private static boolean running;

    private MatchLog() {}

    /** Whether Robot.log() should record anything this run. */
    public static boolean active() {
        return SIM || running;
    }

    /** In init. Never throws: a log that can't be written must not stop the OpMode. */
    public static void start(String opMode) {
        if (SIM || !RobotConstants.Logging.ENABLED) return;
        try {
            File dir = new File(AppUtil.ROBOT_DATA_DIR, "logs");
            String name = opMode + "-" + new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date(System.currentTimeMillis()));
            Logger.reset();
            Logger.addDataReceiver(new RLOGWriter(dir.getPath(), name));
            Logger.recordMetadata("OpMode", opMode);
            Logger.start();
            running = true;
        } catch (RuntimeException | LinkageError e) {
            running = false;
        }
    }

    public static void beginLoop() {
        if (running) Logger.periodicBeforeUser();
    }

    public static void endLoop() {
        if (running) Logger.periodicAfterUser(0, 0);
    }

    public static void stop() {
        if (!running) return;
        running = false;
        Logger.end();
    }

    /**
     * A Pedro pose as AdvantageScope draws FTC fields ("center/rotated"): meters from the field center, x toward the
     * audience, y away from the red wall, heading turned a quarter turn. The sim uses the same frame.
     */
    public static Pose2d fieldPose(Pose pose) {
        return new Pose2d((72 - pose.y()) * METERS_PER_INCH, (pose.x() - 72) * METERS_PER_INCH,
                new Rotation2d(pose.heading() + Math.PI / 2));
    }

    private static boolean classExists(String name) {
        try {
            Class.forName(name, false, MatchLog.class.getClassLoader());
            return true;
        } catch (ClassNotFoundException | LinkageError e) {
            return false;
        }
    }
}
