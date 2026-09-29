package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_HIVE_LEFT_CELL;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.RED_HIVE_RIGHT_CELL;

import com.pedropathing.math.Pose;

/**
 * Which cell of our HIVE is up (the one to shoot into), as seen from our driver station.
 * Both alliances start with the RIGHT cell up, and every tip flips it. The current side lives in MatchState.hive.
 */
public enum Hive {
    LEFT,
    RIGHT;

    /** Where to aim for this cell. */
    public Pose cell(Alliance alliance) {
        return alliance.apply(this == LEFT ? RED_HIVE_LEFT_CELL : RED_HIVE_RIGHT_CELL);
    }

    public Hive flipped() {
        return this == LEFT ? RIGHT : LEFT;
    }
}
