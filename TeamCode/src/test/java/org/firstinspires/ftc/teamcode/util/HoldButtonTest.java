package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class HoldButtonTest {
    private final HoldButton button = new HoldButton(1.0);

    @Test
    public void tapFiresOnRelease() {
        assertEquals(HoldButton.Event.NONE, button.update(true, 0.0));
        assertEquals(HoldButton.Event.NONE, button.update(true, 0.3));
        assertEquals(HoldButton.Event.TAP, button.update(false, 0.4));
        assertEquals(HoldButton.Event.NONE, button.update(false, 0.5));
    }

    @Test
    public void holdFiresOnceWhileHeldAndNoTapAfter() {
        button.update(true, 0.0);
        assertEquals(HoldButton.Event.NONE, button.update(true, 0.99));
        assertEquals(HoldButton.Event.HOLD, button.update(true, 1.0));
        assertEquals(HoldButton.Event.NONE, button.update(true, 2.0));
        assertEquals(HoldButton.Event.NONE, button.update(false, 2.1));
    }

    @Test
    public void eachPressIsSeparate() {
        button.update(true, 0.0);
        button.update(true, 1.2);
        button.update(false, 1.3);
        assertEquals(HoldButton.Event.NONE, button.update(true, 5.0));
        assertEquals(HoldButton.Event.TAP, button.update(false, 5.2));
    }
}
