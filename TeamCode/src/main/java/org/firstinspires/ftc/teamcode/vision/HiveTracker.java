package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Hive;

/**
 * Decides when the camera has seen enough to change MatchState.hive. No hardware, so it's unit tested.
 *
 * - Counts new camera frames only: the loop runs faster than the camera, and the same frame must not count twice.
 * - Switches after framesToSwitch new frames in a row that show the other cell up. A frame showing the current
 *   state, or our tags disagreeing, starts the count over. A frame without our tags changes nothing: the tags face
 *   away after a tip, so missing tags are never evidence.
 * - For holdoffSeconds after the operator sets the hive by hand, the camera must also disagree steadily for
 *   manualDisagreeSeconds before it overrules them.
 * - While locked it never switches.
 */
public final class HiveTracker {
    private final int framesToSwitch;
    private final double holdoffSeconds;
    private final double manualDisagreeSeconds;

    private long lastFrame = Long.MIN_VALUE;
    private int run = 0;
    private double disagreeSince = Double.NaN;
    private double manualAt = Double.NEGATIVE_INFINITY;
    private boolean locked = false;

    public HiveTracker(int framesToSwitch, double holdoffSeconds, double manualDisagreeSeconds) {
        this.framesToSwitch = framesToSwitch;
        this.holdoffSeconds = holdoffSeconds;
        this.manualDisagreeSeconds = manualDisagreeSeconds;
    }

    /** The operator just set the hive. */
    public void manualSet(double now) {
        manualAt = now;
        reset();
    }

    public void setLocked(boolean locked) {
        this.locked = locked;
        reset();
    }

    public boolean isLocked() {
        return locked;
    }

    /** How many frames in a row have shown the other state so far. */
    public int getRun() {
        return run;
    }

    /**
     * @param frameId  anything that changes with every new camera frame (e.g. its arrival time)
     * @param seen     which cell the frame says is up, or null if none of our tags were in it or they disagreed
     * @param conflict whether our tags in the frame disagreed
     * @param current  MatchState.hive right now
     * @return the side to switch to, or null to leave it
     */
    public Hive onFrame(double now, long frameId, Hive seen, boolean conflict, Hive current) {
        if (frameId == lastFrame) return null;
        lastFrame = frameId;
        if (conflict || seen == current) {
            reset();
            return null;
        }
        if (seen == null) return null;

        run++;
        if (Double.isNaN(disagreeSince)) disagreeSince = now;
        if (locked || run < framesToSwitch) return null;
        if (now - manualAt < holdoffSeconds && now - disagreeSince < manualDisagreeSeconds) return null;
        reset();
        return seen;
    }

    private void reset() {
        run = 0;
        disagreeSince = Double.NaN;
    }
}
