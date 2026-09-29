package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Alliance;

/**
 * Formatting for telemetry in HTML mode: telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML).
 * The Driver Station renders it with Android's Html.fromHtml, so only its tags work (b, i, big, small, tt, font color),
 * and runs of spaces collapse into one: use {@link #NBSP} to line things up.
 * tools/telemetry-preview.html mirrors TeleOp's layout; keep it in step when the layout changes.
 */
public final class Html {
    public static final String RED = "#FF5555";
    public static final String BLUE = "#5599FF";
    public static final String GREEN = "#55DD55";
    public static final String YELLOW = "#FFCC33";
    public static final String GRAY = "#999999";
    public static final String DARK_GRAY = "#444444";
    public static final String PINK = "#FF77CC";
    public static final String CYAN = "#44DDEE";

    public static final String NBSP = "&nbsp;";
    /** Space between items on one line (a run of plain spaces would collapse into one). */
    public static final String GAP = NBSP + NBSP + NBSP;

    private Html() {}

    public static String bold(String text) {
        return "<b>" + text + "</b>";
    }

    /** Monospace, so the field diagram and columns of numbers line up. */
    public static String mono(String text) {
        return "<tt>" + text + "</tt>";
    }

    public static String color(String text, String color) {
        return "<font color='" + color + "'>" + text + "</font>";
    }

    public static String allianceColor(Alliance alliance) {
        return alliance == Alliance.RED ? RED : BLUE;
    }

    public static String alliance(Alliance alliance) {
        return bold(color(alliance.toString(), allianceColor(alliance)));
    }
}
