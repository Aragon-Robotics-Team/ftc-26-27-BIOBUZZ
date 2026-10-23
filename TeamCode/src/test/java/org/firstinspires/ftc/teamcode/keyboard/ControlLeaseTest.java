package org.firstinspires.ftc.teamcode.keyboard;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ControlLeaseTest {
    private final ControlLease<String> lease = new ControlLease<>();

    @Test
    public void firstPageControlsAndOthersWatch() {
        assertTrue(lease.input("a", 0));
        assertFalse(lease.input("b", 10));
        assertTrue(lease.input("a", 20));
        assertEquals("a", lease.holder());
    }

    @Test
    public void quietInputPutsPadsAtRestOnce() {
        lease.input("a", 0);
        assertFalse(lease.tick(ControlLease.WATCHDOG_MS - 1));
        assertTrue(lease.tick(ControlLease.WATCHDOG_MS));
        assertFalse(lease.tick(ControlLease.WATCHDOG_MS + 50));
        assertEquals("a", lease.holder());
        // Input again arms the watchdog again
        lease.input("a", 1000);
        assertTrue(lease.tick(1000 + ControlLease.WATCHDOG_MS));
    }

    @Test
    public void longSilenceFreesTheLease() {
        lease.input("a", 0);
        lease.tick(ControlLease.WATCHDOG_MS);
        lease.tick(ControlLease.RELEASE_MS);
        assertNull(lease.holder());
        assertTrue(lease.input("b", ControlLease.RELEASE_MS + 10));
    }

    @Test
    public void takeOverMovesTheLease() {
        lease.input("a", 0);
        lease.takeOver("b", 10);
        assertFalse(lease.input("a", 20));
        assertTrue(lease.input("b", 20));
    }

    @Test
    public void onlyTheHolderReleases() {
        lease.input("a", 0);
        assertFalse(lease.release("b"));
        assertTrue(lease.release("a"));
        assertNull(lease.holder());
        assertFalse(lease.tick(ControlLease.WATCHDOG_MS));
    }
}
