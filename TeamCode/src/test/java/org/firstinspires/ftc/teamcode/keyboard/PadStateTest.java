package org.firstinspires.ftc.teamcode.keyboard;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PadStateTest {
    @Test
    public void parsesBothPads() {
        int bits = (1 << PadState.Button.Y.ordinal()) | (1 << PadState.Button.DPAD_LEFT.ordinal());
        PadState[] pads = PadState.parsePair("0.707,-0.707,-1,0,1,0," + bits + ";0,0,0,0,0,0,0");
        assertEquals(0.707f, pads[0].leftStickX, 1e-6);
        assertEquals(-0.707f, pads[0].leftStickY, 1e-6);
        assertEquals(-1f, pads[0].rightStickX, 1e-6);
        assertEquals(1f, pads[0].leftTrigger, 1e-6);
        assertTrue(pads[0].pressed(PadState.Button.Y));
        assertTrue(pads[0].pressed(PadState.Button.DPAD_LEFT));
        assertFalse(pads[0].pressed(PadState.Button.A));
        assertEquals(0, pads[1].buttons);
    }

    @Test
    public void clampsOutOfRangeValues() {
        PadState pad = PadState.parse("5,-5,0,0,-1,2,0");
        assertEquals(1f, pad.leftStickX, 0);
        assertEquals(-1f, pad.leftStickY, 0);
        assertEquals(0f, pad.leftTrigger, 0); // triggers don't go negative
        assertEquals(1f, pad.rightTrigger, 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNaN() {
        PadState.parse("NaN,0,0,0,0,0,0");
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsUnknownButtons() {
        PadState.parse("0,0,0,0,0,0," + (1 << PadState.Button.values().length));
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsOnePad() {
        PadState.parsePair("0,0,0,0,0,0,0");
    }

    @Test(expected = NumberFormatException.class)
    public void rejectsGarbage() {
        PadState.parse("a,0,0,0,0,0,0");
    }
}
