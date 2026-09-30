package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.firstinspires.ftc.teamcode.Hive;
import org.junit.Test;

public class HiveTrackerTest {
    private static final double FRAME = 0.02; // 50 fps

    private final HiveTracker tracker = new HiveTracker(3, 2.0, 1.0);
    private long frameId = 0;
    private double t = 10;

    /** One new camera frame; returns what the tracker decides. */
    private Hive frame(Hive seen, Hive current) {
        t += FRAME;
        return tracker.onFrame(t, ++frameId, seen, false, current);
    }

    @Test
    public void switchesAfterThreeFrames() {
        assertNull(frame(Hive.LEFT, Hive.RIGHT));
        assertNull(frame(Hive.LEFT, Hive.RIGHT));
        assertEquals(Hive.LEFT, frame(Hive.LEFT, Hive.RIGHT));
    }

    @Test
    public void theSameFrameCountsOnce() {
        frame(Hive.LEFT, Hive.RIGHT);
        assertNull(tracker.onFrame(t + 0.005, frameId, Hive.LEFT, false, Hive.RIGHT));
        assertNull(tracker.onFrame(t + 0.01, frameId, Hive.LEFT, false, Hive.RIGHT));
        assertEquals(1, tracker.getRun());
    }

    @Test
    public void agreeingFrameStartsOver() {
        frame(Hive.LEFT, Hive.RIGHT);
        frame(Hive.LEFT, Hive.RIGHT);
        assertNull(frame(Hive.RIGHT, Hive.RIGHT));
        assertNull(frame(Hive.LEFT, Hive.RIGHT));
        assertNull(frame(Hive.LEFT, Hive.RIGHT));
        assertEquals(Hive.LEFT, frame(Hive.LEFT, Hive.RIGHT));
    }

    @Test
    public void conflictStartsOver() {
        frame(Hive.LEFT, Hive.RIGHT);
        frame(Hive.LEFT, Hive.RIGHT);
        t += FRAME;
        assertNull(tracker.onFrame(t, ++frameId, null, true, Hive.RIGHT));
        assertEquals(0, tracker.getRun());
    }

    @Test
    public void framesWithoutOurTagsChangeNothing() {
        frame(Hive.LEFT, Hive.RIGHT);
        frame(Hive.LEFT, Hive.RIGHT);
        for (int i = 0; i < 20; i++) assertNull(frame(null, Hive.RIGHT));
        assertEquals(Hive.LEFT, frame(Hive.LEFT, Hive.RIGHT));
    }

    @Test
    public void lockedNeverSwitches() {
        tracker.setLocked(true);
        for (int i = 0; i < 50; i++) assertNull(frame(Hive.LEFT, Hive.RIGHT));
    }

    @Test
    public void handSetStateHoldsForASecondOfDisagreement() {
        tracker.manualSet(t);
        double start = t;
        Hive decided = null;
        while (decided == null && t - start < 3) decided = frame(Hive.LEFT, Hive.RIGHT);
        assertEquals(Hive.LEFT, decided);
        assertEquals(1.0, t - start, 2 * FRAME + 1e-9); // switched about 1 s in, not after 3 frames
    }

    @Test
    public void afterTheHoldoffThreeFramesAreEnoughAgain() {
        tracker.manualSet(t);
        t += 2.5;
        assertNull(frame(Hive.LEFT, Hive.RIGHT));
        assertNull(frame(Hive.LEFT, Hive.RIGHT));
        assertEquals(Hive.LEFT, frame(Hive.LEFT, Hive.RIGHT));
    }
}
