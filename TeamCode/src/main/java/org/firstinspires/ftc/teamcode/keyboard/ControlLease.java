package org.firstinspires.ftc.teamcode.keyboard;

/**
 * Which keyboard page drives the robot, and the watchdog on its input. The first page to send input gets the lease;
 * other pages only watch until they take over or the holder goes quiet for {@link #RELEASE_MS}.
 * Plain Java (times are passed in) so it can be unit tested off the robot.
 */
final class ControlLease<C> {
    /** The page sends every 20 ms; this much silence means the laptop, its Wi-Fi or the tab is gone. */
    static final long WATCHDOG_MS = 250;
    /** This much silence frees the lease for another page. */
    static final long RELEASE_MS = 2000;

    private C holder;
    private long lastInputMs;
    /** The gamepads were put at rest since the holder's last input, so the watchdog doesn't do it again. */
    private boolean atRest = true;

    /** Input from a page. True if that page holds the lease (claiming it if free), so its input should be used. */
    synchronized boolean input(C client, long nowMs) {
        if (holder == null) holder = client;
        if (holder != client) return false;
        lastInputMs = nowMs;
        atRest = false;
        return true;
    }

    /** Hands the lease to this page. The caller puts the gamepads at rest, so the old holder's keys don't linger. */
    synchronized void takeOver(C client, long nowMs) {
        holder = client;
        lastInputMs = nowMs;
        atRest = true;
    }

    /** Frees the lease if this page holds it. True if it did: the caller puts the gamepads at rest. */
    synchronized boolean release(C client) {
        if (holder != client) return false;
        holder = null;
        atRest = true;
        return true;
    }

    synchronized void clear() {
        holder = null;
        atRest = true;
    }

    /**
     * Watchdog, run every few ms. True (once) when the holder's input has been quiet for {@link #WATCHDOG_MS}:
     * the caller puts the gamepads at rest. Quiet for {@link #RELEASE_MS} also frees the lease.
     */
    synchronized boolean tick(long nowMs) {
        if (holder == null) return false;
        long quiet = nowMs - lastInputMs;
        if (quiet >= RELEASE_MS) holder = null;
        if (atRest || quiet < WATCHDOG_MS) return false;
        atRest = true;
        return true;
    }

    synchronized C holder() {
        return holder;
    }
}
