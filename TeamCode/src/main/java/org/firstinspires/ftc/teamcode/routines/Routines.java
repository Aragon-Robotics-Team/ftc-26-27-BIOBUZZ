package org.firstinspires.ftc.teamcode.routines;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

/** Every auto the drivers can pick from, in menu order. Register new routines here. */
public enum Routines {
    CYCLE_GARDEN_PARK(CycleGardenPark::new),
    EXAMPLE(ExampleRoutine::new);

    private interface Factory {
        AutoRoutine create(Robot robot, Alliance alliance);
    }

    private final Factory factory;

    Routines(Factory factory) {
        this.factory = factory;
    }

    public AutoRoutine create(Robot robot, Alliance alliance) {
        return factory.create(robot, alliance);
    }

    public Routines next() {
        return values()[(ordinal() + 1) % values().length];
    }

    public Routines previous() {
        return values()[(ordinal() + values().length - 1) % values().length];
    }
}
