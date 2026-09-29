package org.firstinspires.ftc.teamcode.routines;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

/**
 * Every auto the drivers can pick from, in menu order. Register new routines here.
 * Put {@code @Disabled} on a routine class to hide it from the menu, same as for an OpMode.
 */
public enum Routines {
    CYCLE_GARDEN_PARK(CycleGardenPark.class),
    EXAMPLE(ExampleRoutine.class);

    private final Class<? extends AutoRoutine> type;

    Routines(Class<? extends AutoRoutine> type) {
        this.type = type;
    }

    /** Routines shown in the menu, i.e. not {@code @Disabled}. */
    public static List<Routines> enabled() {
        List<Routines> enabled = new ArrayList<>();
        for (Routines routine : values()) {
            if (!routine.type.isAnnotationPresent(Disabled.class)) enabled.add(routine);
        }
        return enabled;
    }

    /** Every routine needs a public (Robot, Alliance) constructor. */
    public AutoRoutine create(Robot robot, Alliance alliance) {
        try {
            return type.getConstructor(Robot.class, Alliance.class).newInstance(robot, alliance);
        } catch (InvocationTargetException e) {
            throw new RuntimeException("Building " + type.getSimpleName() + " failed", e.getCause());
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException(type.getSimpleName() + " needs a public (Robot, Alliance) constructor", e);
        }
    }
}
