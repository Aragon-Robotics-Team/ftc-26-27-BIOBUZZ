package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Alliance;

/** Formatting for telemetry in HTML mode: telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML). */
public final class Html {
    public static final String RED = "#FF5555";
    public static final String BLUE = "#5599FF";
    public static final String GREEN = "#55DD55";
    public static final String YELLOW = "#FFCC33";
    public static final String GRAY = "#999999";

    private Html() {}

    public static String bold(String text) {
        return "<b>" + text + "</b>";
    }

    public static String color(String text, String color) {
        return "<font color='" + color + "'>" + text + "</font>";
    }

    public static String alliance(Alliance alliance) {
        return bold(color(alliance.toString(), alliance == Alliance.RED ? RED : BLUE));
    }
}
