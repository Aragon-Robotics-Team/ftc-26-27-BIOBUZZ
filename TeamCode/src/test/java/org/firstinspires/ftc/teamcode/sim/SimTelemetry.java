package org.firstinspires.ftc.teamcode.sim;

import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Reads values back out of what an OpMode sent to the Driver Station in a sim run. */
final class SimTelemetry {
    private static final Pattern POSE = Pattern.compile("^Pose : \\((-?[\\d.]+), (-?[\\d.]+), (-?[\\d.]+)\\)");
    private static final Pattern PLACE = Pattern.compile("Place robot at : x (-?[\\d.]+) {2}y (-?[\\d.]+) {2}h (-?[\\d.]+)°");

    private SimTelemetry() {}

    /** Pedro's pose {x, y, heading} from the Drivetrain's "Pose" line in the last transmission that has one. */
    static double[] lastPose(List<List<String>> transmissions) {
        return last(transmissions, POSE, "Pose");
    }

    /** Auto's "Place robot at" {x, y, heading in degrees} from the last transmission that has one. */
    static double[] placeRobotAt(List<List<String>> transmissions) {
        return last(transmissions, PLACE, "Place robot at");
    }

    /** The last line sent that starts with this caption, e.g. "Routine". */
    static String lastLine(List<List<String>> transmissions, String caption) {
        for (int i = transmissions.size() - 1; i >= 0; i--) {
            for (String line : transmissions.get(i)) {
                if (line.startsWith(caption + " : ")) return line;
            }
        }
        throw new AssertionError("no \"" + caption + "\" line in the telemetry");
    }

    private static double[] last(List<List<String>> transmissions, Pattern pattern, String what) {
        for (int i = transmissions.size() - 1; i >= 0; i--) {
            for (String line : transmissions.get(i)) {
                Matcher m = pattern.matcher(line);
                if (m.find()) {
                    return new double[] {Double.parseDouble(m.group(1)), Double.parseDouble(m.group(2)),
                            Double.parseDouble(m.group(3))};
                }
            }
        }
        throw new AssertionError("no \"" + what + "\" line in the telemetry");
    }
}
