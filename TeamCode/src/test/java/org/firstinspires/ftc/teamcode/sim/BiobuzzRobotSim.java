package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Hive;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.Auto;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.Ball;
import org.horizon36596.simloop.config.DrivetrainSimConfig;
import org.horizon36596.simloop.config.MechanismSimConfig;
import org.horizon36596.simloop.config.SimRobotConfig;
import org.horizon36596.simloop.fakehardware.FakeGoBildaPinpoint;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeLimelight3A;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.horizon36596.simloop.field.FootprintContact;
import org.horizon36596.simloop.field.GamePieceTracker;
import org.horizon36596.simloop.field.ObstacleBox;
import org.horizon36596.simloop.opmode.GamepadScript;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.plant.MecanumDrivePlant;
import org.horizon36596.simloop.plant.Mechanism1DofPlant;
import org.horizon36596.simloop.simclock.SimClockLoader;
import org.horizon36596.simloop.vision.CameraMount;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The BIOBUZZ robot in simulation: fake devices under the robot config's names, and the physics behind them.
 * Tests build one, place it, and run an OpMode with {@link #runner}; gamepad scripts go through {@link #drivers}, which
 * is how the match clock knows when the OpMode starts:
 * <pre>
 *   BiobuzzRobotSim sim = new BiobuzzRobotSim();
 *   sim.placeAt(56, 8, Math.PI / 2);
 *   sim.runner(Auto.class).initLoopTicks(30).activeTicks(1500).gamepads(sim.drivers(script)).run();
 * </pre>
 *
 * Modelled: the mecanum chassis (rolling the Pinpoint's pods), the turret (through its motor-shaft encoder), the
 * flywheel (the hub's velocity loop on the motor's own spin-up) and the Limelight on the turret, seeing both hives'
 * AprilTags (SimHives) with the cells up that the constructor was given. The balls ({@link #field}): the match's
 * staged balls (SimField), the robot's 4 preload pollen, the intake taking balls and the launcher shooting them
 * (SimLauncher), and the hive trays tipping from the weight in their up cells (SimHiveTrays). The score and clock
 * ({@link #match}), and optionally a scripted alliance partner ({@link #withPartner}). The tags stay where they
 * started when a tray tips. The ball camera ({@link #webcam}) draws what it sees for BallVision's real pipeline once an
 * OpMode enables it.
 *
 * Numbers marked PLACEHOLDER are estimates, not measurements. Replace them as they're measured.
 *
 * The OpMode runs in its own class loader (forTeamClass), so it gets its own copy of RobotConstants and Pedro's
 * Constants. This class reads the defaults from the source; changing a value at runtime here won't reach the OpMode.
 */
public final class BiobuzzRobotSim {
    // ---- Drivetrain: goBILDA 312 RPM motors on 104 mm mecanum wheels ----
    private static final double DRIVE_TICKS_PER_REV = 537.7;
    private static final double DRIVE_MAX_TICKS_PER_SECOND = 312 / 60.0 * DRIVE_TICKS_PER_REV; // free speed
    private static final double WHEEL_RADIUS = 104 / 25.4 / 2; // inches
    private static final double DRIVE_TICKS_PER_INCH = DRIVE_TICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS);
    private static final double DRIVE_RATE = 1 / 0.2; // 1/s: the 0.2 s speed-up time constant in pedro/Constants. PLACEHOLDER
    private static final double TRACK_WIDTH = 12.36; // inches between left and right wheel centers (CAD master assembly, Oct 26)
    private static final double WHEEL_BASE = 7.84;   // inches between front and back wheel centers (CAD master assembly, Oct 26)

    // ---- Turret: goBILDA 1150 RPM motor, Through Bore on its shaft, 30T:100T to the ring ----
    private static final double TURRET_MOTOR_RPM = 1150;
    private static final double TURRET_TICKS_PER_RADIAN =
            RobotConstants.Launcher.TURRET_MOTOR_TICKS_PER_REV * RobotConstants.Launcher.TURRET_GEAR_RATIO / (2 * Math.PI);
    private static final double TURRET_TIME_CONSTANT = 0.05; // seconds. PLACEHOLDER
    // Hard stops, past the code's soft limits (TURRET_MIN/MAX_ANGLE). PLACEHOLDER: measure how far the turret can go
    private static final double TURRET_HARD_STOP_MARGIN = Math.toRadians(20);

    // ---- Flywheel: goBILDA 6000 RPM, 28 ticks/rev ----
    // Flat-out speed at 12 V with the counter-rollers on, as the flywheel tuner measured it: FlywheelController's
    // feedforward is FLYWHEEL_KV power per tick/s, so full power is 1 / FLYWHEEL_KV (about 1850 ticks/s). Not
    // FLYWHEEL_MAX_VELOCITY, the most the code asks for, and not the motor's 2800 ticks/s free speed
    private static final double FLYWHEEL_MAX_TICKS_PER_SECOND = 1 / RobotConstants.Launcher.FLYWHEEL_KV;
    private static final double FLYWHEEL_RATE = 1 / 0.4; // 1/s: spin-up time constant 0.4 s. PLACEHOLDER: time a spin-up

    private static final double INTAKE_MAX_TICKS_PER_SECOND = 2800; // PLACEHOLDER: nothing reads it back

    // ---- Limelight 3A: on the turret, 4.8 in ahead of its axis, lens 12.2 in up, tilted up (RobotConstants.Vision) ----
    private static final double LIMELIGHT_AHEAD_OF_TURRET_AXIS = 4.8; // inches (CAD, Oct 26)
    private static final double LIMELIGHT_H_FOV = 54.5, LIMELIGHT_V_FOV = 42; // degrees, Limelight 3A spec

    private static final int PRELOAD = 4; // pollen each robot starts with (Competition Manual)
    private int preload = PRELOAD;
    private boolean preloaded;
    // scatter(): balls scattered over the tiles instead of the match's staging
    private long scatterSeed;
    private int scatterPollen = -1, scatterNectar;
    private List<double[]> scattered = Collections.emptyList();

    private static final DrivetrainSimConfig DRIVETRAIN = new DrivetrainSimConfig() {
        @Override public double trackWidth() { return TRACK_WIDTH; }
        @Override public double wheelBase() { return WHEEL_BASE; }
        @Override public double wheelRadius() { return WHEEL_RADIUS; }
        @Override public double ticksPerInch() { return DRIVE_TICKS_PER_INCH; }
        @Override public double maxVelocityTicksPerSecond() { return DRIVE_MAX_TICKS_PER_SECOND; }
        @Override public double maxAccel() { return DRIVE_RATE; }
        // How the motors are bolted in, [FL, FR, BL, BR]: the left pair is mirrored, which is why pedro/Constants
        // reverses it in software. If the two disagree, the sim robot spins instead of driving, like the real one would.
        @Override public double[] wheelMountingSigns() { return new double[] {-1, 1, -1, 1}; }
        @Override public double fieldHalfWidth() { return 72; }
        @Override public double fieldHalfHeight() { return 72; }
    };

    private static final MechanismSimConfig TURRET = new MechanismSimConfig() {
        @Override public double timeConstant() { return TURRET_TIME_CONSTANT; }
        @Override public double maxSpeed() { return TURRET_MOTOR_RPM / 60 * 2 * Math.PI / RobotConstants.Launcher.TURRET_GEAR_RATIO; } // rad/s
        @Override public double minPosition() { return RobotConstants.Launcher.TURRET_MIN_ANGLE - TURRET_HARD_STOP_MARGIN; }
        @Override public double maxPosition() { return RobotConstants.Launcher.TURRET_MAX_ANGLE + TURRET_HARD_STOP_MARGIN; }
        @Override public double gravityHoldPowerFraction() { return 0; } // turns flat
    };

    /** For runLogged: names the run and its drivetrain. */
    public static final SimRobotConfig ROBOT_CONFIG = new SimRobotConfig() {
        @Override public String[] driveMotorNames() { return DRIVE_MOTOR_NAMES; }
        @Override public String odometryName() { return Constants.localizerConfig.name.get(); }
        @Override public DrivetrainSimConfig drivetrain() { return DRIVETRAIN; }
    };

    /** [FL, FR, BL, BR], the order the drive plant reads them. */
    private static final String[] DRIVE_MOTOR_NAMES = {
            Constants.drivetrainConfig.frontLeftName.get(), Constants.drivetrainConfig.frontRightName.get(),
            Constants.drivetrainConfig.backLeftName.get(), Constants.drivetrainConfig.backRightName.get()};

    public final FakeHardwareMap hardwareMap = new FakeHardwareMap();
    public final FakeMotor[] driveMotors = new FakeMotor[4];
    public final FakeGoBildaPinpoint pinpoint = new FakeGoBildaPinpoint();
    public final FakeMotor flywheel = new FakeMotor(FLYWHEEL_MAX_TICKS_PER_SECOND, FLYWHEEL_RATE, 28);
    public final FakeMotor turret = new FakeMotor(TURRET_MOTOR_RPM / 60 * RobotConstants.Launcher.TURRET_MOTOR_TICKS_PER_REV,
            10, RobotConstants.Launcher.TURRET_MOTOR_TICKS_PER_REV);
    public final FakeMotor intake = new FakeMotor(INTAKE_MAX_TICKS_PER_SECOND, 10);
    public final FakeServo gate = new FakeServo();
    public final SimWebcam webcam = new SimWebcam();
    public final FakeVoltageSensor battery = new FakeVoltageSensor(); // 12 V
    public final FakeLimelight3A limelight;
    /** The balls, the hive trays and the flowers. One more than the robot holds: the partner's shots pass through. */
    public final GamePieceTracker field = new GamePieceTracker(SimLauncher.CAPACITY + 1, SimField.world());
    private final SimLauncher launcher = new SimLauncher(field, intake, gate, flywheel);
    private SimMatch.Robot us = new SimMatch.Robot(Alliance.RED);
    private final List<SimMatch.Robot> robots = new ArrayList<>();
    /** The clock and the score. */
    public final SimMatch match;
    private SimPartner partner;
    private double lastFieldX = Double.NaN, lastFieldY;
    /** What the chassis can't drive into, field frame: the four walls (thick boxes just outside the tiles) and the field's obstacles. */
    private final List<ObstacleBox> solid = new ArrayList<>();
    private String touching = "";
    /** Which cell of each hive (red, blue) the Limelight's tags show up: moved when a tray tips. */
    private final Hive[] tagsUp = new Hive[2];
    /**
     * FakeLimelight3A's tag layout. It's final and SimLoop has no setter, so the tags move by reflection when a tray
     * tips: a stopgap until SimLoop lets a layout change during a run.
     */
    private static final java.lang.reflect.Field LIMELIGHT_LAYOUT;
    static {
        try {
            LIMELIGHT_LAYOUT = FakeLimelight3A.class.getDeclaredField("layout");
            LIMELIGHT_LAYOUT.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new ExceptionInInitializerError(e);
        }
    }

    public final MecanumDrivePlant drivePlant;
    public final Mechanism1DofPlant turretPlant;

    /** Both hives with the RIGHT cell up, as every match starts. */
    public BiobuzzRobotSim() {
        this(Hive.RIGHT, Hive.RIGHT);
    }

    /** Which cell of each hive is really up, each as seen from that alliance's driver station. Fixed for the run. */
    public BiobuzzRobotSim(Hive redHiveUp, Hive blueHiveUp) {
        field.setBodyMotion(SimHiveTrays.RED, SimHiveTrays.angle(false, redHiveUp), 0);
        field.setBodyMotion(SimHiveTrays.BLUE, SimHiveTrays.angle(true, blueHiveUp), 0);
        tagsUp[0] = redHiveUp;
        tagsUp[1] = blueHiveUp;
        robots.add(us);
        match = new SimMatch(field, robots);
        solid.add(new ObstacleBox("wallRight", 72, 100, -100, 100, 0, 12));
        solid.add(new ObstacleBox("wallLeft", -100, -72, -100, 100, 0, 12));
        solid.add(new ObstacleBox("wallRed", -100, 100, -100, -72, 0, 12));
        solid.add(new ObstacleBox("wallBlue", -100, 100, 72, 100, 0, 12));
        solid.addAll(field.getWorld().getObstacles());
        for (int i = 0; i < RobotConstants.Field.FLOWERS.length; i++) solid.add(SimField.flowerColumn(i));
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = new FakeMotor(DRIVE_MAX_TICKS_PER_SECOND, DRIVE_RATE, DRIVE_TICKS_PER_REV);
            hardwareMap.register(DRIVE_MOTOR_NAMES[i], driveMotors[i]);
        }

        // The pods as they really sit: the same as pedro/Constants tells the Pinpoint, 4-bar pods
        double toMm = Constants.localizerConfig.offsetUnits.get().toMm(1);
        pinpoint.setPhysicalPodMount(
                Constants.localizerConfig.xPodOffset.get() * toMm, Constants.localizerConfig.yPodOffset.get() * toMm,
                Constants.localizerConfig.xPodDirection.get(), Constants.localizerConfig.yPodDirection.get(),
                FakeGoBildaPinpoint.DEFAULT_TICKS_PER_MM);
        hardwareMap.register(Constants.localizerConfig.name.get(), pinpoint);

        hardwareMap.register(RobotConstants.Launcher.FLYWHEEL_MOTOR_NAME, flywheel);
        hardwareMap.register(RobotConstants.Launcher.TURRET_MOTOR_NAME, turret);
        hardwareMap.register(RobotConstants.Intake.MOTOR_NAME, intake);
        hardwareMap.register(RobotConstants.Gate.SERVO_NAME, gate);
        hardwareMap.register(RobotConstants.BallVision.WEBCAM_NAME, webcam);
        // The Launcher's FlywheelController scales its power by the battery voltage
        hardwareMap.register("Control Hub", battery);

        // The mount is fixed to the turret, so the Limelight is told the turret's pose rather than the chassis's
        CameraMount onTurret = new CameraMount(LIMELIGHT_AHEAD_OF_TURRET_AXIS, 0, RobotConstants.Vision.LENS_HEIGHT, 0,
                -RobotConstants.Vision.TILT); // CameraMount's pitch is positive down
        limelight = FakeLimelight3A.builder(SimHives.layout(redHiveUp, blueHiveUp), onTurret, LIMELIGHT_H_FOV,
                LIMELIGHT_V_FOV).build();
        hardwareMap.register(RobotConstants.Vision.LIMELIGHT_NAME, limelight);

        drivePlant = new MecanumDrivePlant(driveMotors, DRIVETRAIN);
        turretPlant = new Mechanism1DofPlant(turret, TURRET);
        placeAt(72, 72, 0);
        reportTurret(); // the turret port reads the turret from the start, so the OpMode's encoder reset zeroes it here
    }

    /**
     * Puts the robot somewhere on the field, in Pedro coordinates: inches from the field corner, radians CCW.
     * Only the walls care where this is. The Pinpoint reads what the OpMode sets (follower.setPose), plus how far
     * the chassis has moved since, just like on the robot. So place the robot where the OpMode thinks it starts.
     */
    public void placeAt(double x, double y, double heading) {
        drivePlant.setPose(x - 72, y - 72, heading);
        pinpoint.attachChassisAt(drivePlant.getX() * 25.4, drivePlant.getY() * 25.4, drivePlant.getHeading());
        reportLimelight();
    }

    /** Which alliance the robot is on (default red): what it scores for and which hive it aims at. */
    public BiobuzzRobotSim alliance(Alliance alliance) {
        robots.remove(us);
        us = new SimMatch.Robot(alliance);
        robots.add(0, us);
        return this;
    }

    /** Adds a scripted alliance partner (SimPartner) on our alliance: it shoots its preload and parks. After alliance(). */
    public BiobuzzRobotSim withPartner() {
        partner = new SimPartner(us.alliance, field);
        robots.add(partner.robot);
        return this;
    }

    /**
     * Wraps a gamepad script so the match clock starts with the OpMode. Pass every script through this:
     * {@code runner(...).gamepads(sim.drivers(script))}. runner() already sets {@code drivers(GamepadScript.NONE)}.
     */
    public GamepadScript drivers(GamepadScript script) {
        return (phase, tick, gamepad1, gamepad2) -> {
            if (phase == OpModePhase.ACTIVE && tick == 0) match.start();
            script.apply(phase, tick, gamepad1, gamepad2);
        };
    }

    /** Where the robot really is, in Pedro coordinates (inches from the field corner). */
    public double x() { return drivePlant.getX() + 72; }
    public double y() { return drivePlant.getY() + 72; }
    /** Radians CCW, unwrapped. */
    public double heading() { return drivePlant.getHeading(); }
    /** Balls the launcher has fired this run. */
    public int shotsFired() { return launcher.shotsFired(); }
    /** Turret angle, robot-relative radians, CCW positive (same as Launcher). */
    public double turretAngle() { return turretPlant.getPosition(); }

    /**
     * Runs an OpMode on this robot, with Pedro on the sim clock (its Pinpoint localizer sleeps 500 ms in init).
     * Add initLoopTicks, activeTicks and gamepads, then run().
     */
    /** Balls the robot starts holding, instead of the 4 preload pollen. Before runner(). */
    public BiobuzzRobotSim preload(int pollen) {
        preload = pollen;
        return this;
    }

    /**
     * Starts the field with balls scattered over the tiles (SimField.scatter, clear of the robot's start) instead of
     * the match's staging. Seeded: the same seed gives the same field. Before runner().
     */
    public BiobuzzRobotSim scatter(long seed, int pollen, int nectar) {
        scatterSeed = seed;
        scatterPollen = pollen;
        scatterNectar = nectar;
        return this;
    }

    /** Where scatter() put the balls, Pedro inches: {type (0 pollen, 1 red nectar, 2 blue nectar), x, y}. */
    public List<double[]> scattered() {
        return scattered;
    }

    public OpModeRunner runner(Class<? extends OpMode> opMode) {
        if (!preloaded) {
            // The field's balls go in here, not in the constructor, so placeAt() and scatter() come first
            if (scatterPollen >= 0) scattered = SimField.scatter(field, scatterSeed, scatterPollen, scatterNectar, x(), y());
            else SimField.stage(field);
            launcher.preload(Ball.Type.POLLEN, preload);
        }
        preloaded = true;
        match.setAuto(Auto.class.isAssignableFrom(opMode));
        return OpModeRunner.forTeamClass(opMode, hardwareMap)
                .teamPackages("org.firstinspires.ftc.teamcode.", SimClockLoader.PEDRO_PATHING)
                .gamepads(drivers(GamepadScript.NONE))
                .eachTick(this::advancePhysics);
    }

    /** One tick of physics, after the OpMode has set its powers and the fakes have updated (the runner's order). */
    public void advancePhysics(double deltaTime) {
        double headingBefore = drivePlant.getHeading();
        drivePlant.update(deltaTime);
        collide(headingBefore);
        pinpoint.moveChassisTo(drivePlant.getX() * 25.4, drivePlant.getY() * 25.4, drivePlant.getHeading());

        turretPlant.update(deltaTime);
        reportTurret();
        reportLimelight();

        double fx = FieldFrame.x(x(), y()), fy = FieldFrame.y(x(), y()), fh = FieldFrame.heading(heading());
        double vx = Double.isNaN(lastFieldX) ? 0 : (fx - lastFieldX) / deltaTime;
        double vy = Double.isNaN(lastFieldX) ? 0 : (fy - lastFieldY) / deltaTime;
        lastFieldX = fx;
        lastFieldY = fy;
        field.updateRobotPose(fx, fy, fh);
        double[] axis = turretAxis();
        launcher.update(deltaTime, FieldFrame.x(axis[0], axis[1]), FieldFrame.y(axis[0], axis[1]),
                fh + turretAngle(), vx, vy, SimHiveTrays.upOpening(field, us.alliance == Alliance.BLUE));
        if (partner != null) partner.update(deltaTime);
        field.advance(deltaTime);
        moveTagsWithTrays();
        us.x = fx;
        us.y = fy;
        us.heading = fh;
        webcam.tick(field, fx, fy, fh);
        match.update(deltaTime);
        log();
    }

    /**
     * Stops the chassis at the walls, the hive's feet and the flower columns: SimLoop's drive plant only keeps the
     * robot's center on the field. A turn that swings the frame further into something doesn't happen (the heading goes
     * back to where it was this tick), so a robot up against a wall can't turn there. Then the footprint is pushed back
     * out of anything it overlaps (FootprintContact) and loses its speed into it, so it slides along. A few passes, for
     * a corner. The Pinpoint reads the result, as a robot pinned against a wall measures itself standing still.
     */
    private void collide(double headingBefore) {
        touching = "";
        double depth = overlap(drivePlant.getHeading());
        if (depth > 0 && overlap(headingBefore) < depth - 1e-9) {
            drivePlant.setPose(drivePlant.getX(), drivePlant.getY(), headingBefore);
        }
        for (int pass = 0; pass < 3; pass++) {
            double fx = FieldFrame.x(x(), y()), fy = FieldFrame.y(x(), y()), fh = FieldFrame.heading(heading());
            FootprintContact.Push push = null;
            for (ObstacleBox box : solid) {
                push = FootprintContact.pushOutOf(fx, fy, fh, RobotConstants.Field.ROBOT_LENGTH,
                        RobotConstants.Field.ROBOT_WIDTH, box);
                if (push != null) {
                    touching = touching.isEmpty() ? box.getName() : touching + "," + box.getName();
                    break;
                }
            }
            if (push == null) return;
            // Field (dx, dy) in Pedro: x is field +Y, y is field -X. The plant's frame is Pedro moved to the center
            double dx = push.getDyInches(), dy = -push.getDxInches();
            drivePlant.setPose(drivePlant.getX() + dx, drivePlant.getY() + dy, drivePlant.getHeading());
            drivePlant.removeVelocityInto(dx, dy);
        }
    }

    /** How deep the footprint is in everything solid, inches summed, at the plant's position and this heading. */
    private double overlap(double plantHeading) {
        double fx = FieldFrame.x(x(), y()), fy = FieldFrame.y(x(), y()), fh = FieldFrame.heading(plantHeading);
        double depth = 0;
        for (ObstacleBox box : solid) {
            FootprintContact.Push push = FootprintContact.pushOutOf(fx, fy, fh, RobotConstants.Field.ROBOT_LENGTH,
                    RobotConstants.Field.ROBOT_WIDTH, box);
            if (push != null) depth += push.getDepthInches();
        }
        return depth;
    }

    /** Once a tray has tipped to its other stop, its tags are where the Limelight would now see them. */
    private void moveTagsWithTrays() {
        boolean moved = false;
        for (int i = 0; i < 2; i++) {
            Hive up = SimHiveTrays.up(i == 1, field.getBodyAngleRad(i == 0 ? SimHiveTrays.RED : SimHiveTrays.BLUE));
            if (up != null && up != tagsUp[i]) {
                tagsUp[i] = up;
                moved = true;
            }
        }
        if (!moved) return;
        try {
            LIMELIGHT_LAYOUT.set(limelight, SimHives.layout(tagsUp[0], tagsUp[1]));
        } catch (IllegalAccessException e) {
            throw new IllegalStateException("can't move the Limelight's tags", e);
        }
    }

    /**
     * Where the Limelight is: the turret axis (TURRET_OFFSET_X/Y from the robot center), facing the turret's way. In the
     * frame SimHives uses: Pedro coordinates moved to the field center. Takes effect from the next capture.
     */
    private void reportLimelight() {
        double[] axis = turretAxis();
        limelight.setRobotPose(axis[0] - 72, axis[1] - 72, heading() + turretAngle());
    }

    /** The turret axis, Pedro coordinates: TURRET_OFFSET_X/Y from the robot center. */
    private double[] turretAxis() {
        return robotPoint(RobotConstants.Launcher.TURRET_OFFSET_X, RobotConstants.Launcher.TURRET_OFFSET_Y);
    }

    /** A point on the robot (inches forward and left of its center) in Pedro coordinates. */
    private double[] robotPoint(double forward, double left) {
        double h = heading();
        return new double[] {x() + forward * Math.cos(h) - left * Math.sin(h), y() + forward * Math.sin(h) + left * Math.cos(h)};
    }

    /**
     * What AdvantageScope shows for a runLogged run (keys under RealOutputs/). Poses are in meters, in the frame
     * AdvantageScope uses for FTC fields ("center/rotated"), not Pedro's: see {@link FieldFrame}.
     */
    private void log() {
        Logger.recordOutput("Sim/RobotPose", Pose2d.struct, FieldFrame.pose2d(x(), y(), heading()));
        // Where the robot thinks it is: the Pinpoint's last reading, which is the pose Pedro follows
        Logger.recordOutput("Sim/PinpointPose", Pose2d.struct, FieldFrame.pose2d(pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(AngleUnit.RADIANS)));
        Logger.recordOutput("Sim/Touching", touching); // the walls or obstacles the chassis is against, if any
        Logger.recordOutput("Sim/TurretAngleDeg", Math.toDegrees(turretAngle()));
        Logger.recordOutput("Sim/TurretFieldPose", Pose2d.struct, FieldFrame.pose2d(x(), y(), heading() + turretAngle()));
        Logger.recordOutput("Sim/FlywheelTicksPerSec", flywheel.getVelocity());
        // The robot model's moving parts, robot frame, in the order of the Robot_BIOBUZZ asset's components:
        // 0 = the turret and everything on it, about the turret axis
        Logger.recordOutput("RobotModel/components3d", Pose3d.struct, new Pose3d[] {
                new Pose3d(RobotConstants.Launcher.TURRET_OFFSET_X * FieldFrame.METERS_PER_INCH,
                        RobotConstants.Launcher.TURRET_OFFSET_Y * FieldFrame.METERS_PER_INCH, 0,
                        new Rotation3d(0, 0, turretAngle()))});
        // The hive trays, for the "BIOBUZZ Sim (tilting hives)" field, which has no trays of its own; and the balls
        Logger.recordOutput("Hive/red/trayPose3d", Pose3d.struct,
                SimHives.trayPose(false, field.getBodyAngleRad(SimHiveTrays.RED)));
        Logger.recordOutput("Hive/blue/trayPose3d", Pose3d.struct,
                SimHives.trayPose(true, field.getBodyAngleRad(SimHiveTrays.BLUE)));
        field.record("GamePieces");
        launcher.record();
        match.record();
        if (partner != null) partner.record();
        SimField.recordZones();

        // The cameras, field frame, for an AdvantageScope Axes or Camera object. The Limelight turns with the turret;
        // pitch is positive down, so its upward tilt is negative
        double[] axis = turretAxis();
        double turret = heading() + turretAngle();
        Logger.recordOutput("Sim/Camera/limelight", Pose3d.struct, FieldFrame.pose3d(
                axis[0] + LIMELIGHT_AHEAD_OF_TURRET_AXIS * Math.cos(turret),
                axis[1] + LIMELIGHT_AHEAD_OF_TURRET_AXIS * Math.sin(turret),
                RobotConstants.Vision.LENS_HEIGHT, 0, -RobotConstants.Vision.TILT, turret));
        double[] webcam = robotPoint(RobotConstants.BallVision.LENS_FORWARD, RobotConstants.BallVision.LENS_LEFT);
        Logger.recordOutput("Sim/Camera/webcam", Pose3d.struct, FieldFrame.pose3d(webcam[0], webcam[1],
                RobotConstants.BallVision.LENS_HEIGHT, RobotConstants.BallVision.ROLL, RobotConstants.BallVision.TILT,
                heading() + RobotConstants.BallVision.YAW));
    }

    /**
     * The turret encoder is on the motor shaft, so it reads the turret through the gearing. It counts the way
     * Launcher expects (TURRET_ENCODER_REVERSED): the sim can't know the real wiring any better than the code does.
     */
    private void reportTurret() {
        double sign = RobotConstants.Launcher.TURRET_ENCODER_REVERSED ? -1 : 1;
        turret.setEncoderMeasuringAnotherMechanism(
                sign * turretPlant.getPosition() * TURRET_TICKS_PER_RADIAN,
                sign * turretPlant.getVelocity() * TURRET_TICKS_PER_RADIAN);
        turret.setMeasuredShaftSpeedFraction(turretPlant.getVelocityFractionOfMaxSpeed());
    }
}
