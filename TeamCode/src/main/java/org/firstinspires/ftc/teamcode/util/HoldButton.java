package org.firstinspires.ftc.teamcode.util;

/**
 * Tells a tap from a hold on one button. A tap fires on release, so a hold never fires a tap first;
 * a hold fires once, as soon as the button has been down for holdSeconds, while it's still held.
 */
public final class HoldButton {
    public enum Event { NONE, TAP, HOLD }

    private final double holdSeconds;
    private double downAt = Double.NaN;
    private boolean held = false;

    public HoldButton(double holdSeconds) {
        this.holdSeconds = holdSeconds;
    }

    /** Call every loop with the button's state. */
    public Event update(boolean pressed, double now) {
        if (pressed) {
            if (Double.isNaN(downAt)) {
                downAt = now;
                held = false;
            } else if (!held && now - downAt >= holdSeconds) {
                held = true;
                return Event.HOLD;
            }
            return Event.NONE;
        }
        if (Double.isNaN(downAt)) return Event.NONE;
        boolean wasHeld = held;
        downAt = Double.NaN;
        held = false;
        return wasHeld ? Event.NONE : Event.TAP;
    }
}
