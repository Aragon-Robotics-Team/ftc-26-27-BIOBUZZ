package org.firstinspires.ftc.teamcode.keyboard;

/**
 * One gamepad as the keyboard page sends it: "lx,ly,rx,ry,lt,rt,buttons". Sticks and triggers use the SDK's
 * conventions (stick up is -y), and buttons is a bit mask in {@link Button} order, with the SDK's Xbox names
 * (a = cross, b = circle, x = square, y = triangle, back = share, start = options, guide = ps).
 * Plain Java so it can be unit tested off the robot.
 */
final class PadState {
    /** Bit order of the buttons mask; the page (assets/keyboard/index.html) uses the same order. */
    enum Button {
        A, B, X, Y, GUIDE, START, BACK, LEFT_BUMPER, RIGHT_BUMPER,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON
    }

    static final PadState REST = new PadState(0, 0, 0, 0, 0, 0, 0);

    final float leftStickX, leftStickY, rightStickX, rightStickY;
    final float leftTrigger, rightTrigger;
    final int buttons;

    PadState(float leftStickX, float leftStickY, float rightStickX, float rightStickY,
             float leftTrigger, float rightTrigger, int buttons) {
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickX = rightStickX;
        this.rightStickY = rightStickY;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.buttons = buttons;
    }

    boolean pressed(Button button) {
        return (buttons & (1 << button.ordinal())) != 0;
    }

    /** Both gamepads from one input message, "<gamepad 1>;<gamepad 2>". */
    static PadState[] parsePair(String payload) {
        String[] pads = payload.split(";", -1);
        if (pads.length != 2) throw new IllegalArgumentException("expected 2 gamepads: " + payload);
        return new PadState[] {parse(pads[0]), parse(pads[1])};
    }

    static PadState parse(String pad) {
        String[] f = pad.split(",", -1);
        if (f.length != 7) throw new IllegalArgumentException("expected 7 fields: " + pad);
        int buttons = Integer.parseInt(f[6].trim());
        if (buttons >>> Button.values().length != 0) throw new IllegalArgumentException("unknown buttons: " + pad);
        return new PadState(axis(f[0], -1), axis(f[1], -1), axis(f[2], -1), axis(f[3], -1),
                axis(f[4], 0), axis(f[5], 0), buttons);
    }

    /** A number clamped to [min, 1]; NaN and infinity are rejected rather than reaching the motors. */
    private static float axis(String text, float min) {
        float v = Float.parseFloat(text.trim());
        if (Float.isNaN(v) || Float.isInfinite(v)) throw new IllegalArgumentException("not a number: " + text);
        return Math.max(min, Math.min(1, v));
    }
}
