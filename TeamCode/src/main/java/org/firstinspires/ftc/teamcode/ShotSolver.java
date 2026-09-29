package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.Field.HIVE_OPENING_CENTER_Z;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.HIVE_OPENING_HEIGHT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.HIVE_OPENING_TILT;
import static org.firstinspires.ftc.teamcode.RobotConstants.Field.HIVE_OPENING_WIDTH;
import static org.firstinspires.ftc.teamcode.RobotConstants.Launcher.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;

/**
 * Finds the shot into a hive cell by simulating the ball's flight: gravity, air drag and lift from the hood's
 * backspin, launched with the turret pivot's velocity (the ball keeps the robot's motion; the air stays still).
 * The hood is fixed, so the unknowns are exit speed and turret heading:
 * 1. Newton's method adjusts both until the simulated ball crosses the middle of the opening.
 * 2. The speed is then moved to the middle of the range of speeds that still score. With a fixed hood that is the most
 *    reliable arc: it leaves the most room for flywheel error either way.
 * The flight model and numbers come from BiobuzzSim (.trials/launcher_real.py, docs/launcher-design-numbers.md).
 *
 * Solved every loop from the last answer, so it usually takes one Newton step. Not thread-safe; one per Launcher.
 */
public class ShotSolver {
    /** A solved shot. Angles in radians, distances in inches, speeds in inches/sec. */
    public static class Shot {
        /** False if no exit speed within the flywheel's range gets the ball in from here. */
        public final boolean feasible;
        /** Ball speed out of the launcher. */
        public final double exitSpeed;
        /** Flywheel ticks/sec for exitSpeed, before the operator's nudge. */
        public final double flywheelVelocity;
        /** Robot-relative turret angle. */
        public final double turretAngle;
        /** How fast turretAngle is changing as the robot moves (radians/sec), for the turret feedforward. */
        public final double turretRate;
        /** Exit speed can be off by this much either way and still score. */
        public final double speedTolerance;
        /** Turret angle can be off by this much either way and still score. */
        public final double angleTolerance;
        /** Seconds from launch to the opening. */
        public final double timeOfFlight;
        /** How far the turret aims off the target to allow for the robot's motion (radians). */
        public final double leadAngle;
        /** Horizontal distance from the turret pivot to the center of the opening. */
        public final double distance;

        Shot(boolean feasible, double exitSpeed, double turretAngle, double turretRate, double speedTolerance,
             double angleTolerance, double timeOfFlight, double leadAngle, double distance) {
            this.feasible = feasible;
            this.exitSpeed = exitSpeed;
            this.flywheelVelocity = Math.min(exitSpeed * FLYWHEEL_TICKS_PER_EXIT_SPEED, FLYWHEEL_MAX_VELOCITY);
            this.turretAngle = turretAngle;
            this.turretRate = turretRate;
            this.speedTolerance = speedTolerance;
            this.angleTolerance = angleTolerance;
            this.timeOfFlight = timeOfFlight;
            this.leadAngle = leadAngle;
            this.distance = distance;
        }
    }

    private static final double GRAVITY = 386.09;     // in/s²
    private static final double METERS_PER_INCH = 0.0254;
    private static final double STEP = 0.01;          // integration step (s); RK4, so this is plenty
    private static final double MAX_FLIGHT = 2.0;     // s
    private static final double SPEED_PROBE = 0.5;    // in/s, for the Newton derivatives
    private static final double YAW_PROBE = 0.002;    // rad
    private static final double CONVERGED = 0.05;     // in from the middle of the opening
    private static final int MAX_ITERATIONS = 8;

    // ---- The scene for the current solve, set by setUp() ----
    private double pivotX, pivotY;          // launch point (field inches)
    private double ballVx, ballVy;          // velocity the ball inherits from the robot
    private double centerX, centerY;        // center of the opening (at HIVE_OPENING_CENTER_Z)
    private double normalX, normalY, normalZ; // opening's face normal, pointing out toward the robot
    private double upX, upY, upZ;           // along the face toward its upper lip
    private double sideX, sideY;            // along the face sideways (horizontal)
    private double k;                       // ½ρA/m, per inch

    // ---- Result of the last fly(), in fields so the loop doesn't allocate ----
    private double hitUp, hitSide, hitTime;
    private double spinX, spinY;            // backspin axis of the current flight
    private double ax, ay, az;              // accel() output

    // ---- Warm start and per-loop cache ----
    private double lastSpeed = Double.NaN;
    private double lastYaw;
    private final double[] cacheKey = new double[9];
    private Shot cached = null;

    /**
     * Solve for a shot at a hive cell.
     *
     * @param robotVelocity field-relative (inches/sec and radians/sec), as {@code Follower.velocity()} gives it
     * @param target        center of the cell's opening, with its heading pointing out of the opening
     */
    public Shot solve(Pose robotPose, Velocity robotVelocity, Pose target) {
        // The turret and flywheel commands both ask every loop; only solve once
        double[] key = {robotPose.x(), robotPose.y(), robotPose.heading(), robotVelocity.vx, robotVelocity.vy,
                robotVelocity.omega, target.x(), target.y(), target.heading()};
        if (cached != null && java.util.Arrays.equals(key, cacheKey)) return cached;
        System.arraycopy(key, 0, cacheKey, 0, key.length);
        cached = solveUncached(robotPose, robotVelocity, target);
        return cached;
    }

    private Shot solveUncached(Pose robotPose, Velocity robotVelocity, Pose target) {
        // Aim from where the robot will be once this loop's commands take effect
        Pose pose = new Pose(
                robotPose.x() + robotVelocity.vx * AIM_LOOKAHEAD,
                robotPose.y() + robotVelocity.vy * AIM_LOOKAHEAD,
                robotPose.heading() + robotVelocity.omega * AIM_LOOKAHEAD);
        setUp(pose, robotVelocity, target);

        double dx = centerX - pivotX;
        double dy = centerY - pivotY;
        double distance = Math.max(Math.hypot(dx, dy), 1);
        double directYaw = Math.atan2(dy, dx);

        double speed = Double.isNaN(lastSpeed) ? firstGuess(distance) : lastSpeed;
        double yaw = Double.isNaN(lastSpeed) ? directYaw : lastYaw;

        // 1. Newton: hit the middle of the opening. dUp/dSpeed and dSide/dYaw are kept for the tolerances.
        boolean converged = false;
        double upPerSpeed = 0, sidePerYaw = 0;
        for (int i = 0; i < MAX_ITERATIONS && !converged; i++) {
            if (!fly(speed, yaw)) {
                speed *= 1.1; // fell short of the opening's plane: go faster
                continue;
            }
            double up = hitUp, side = hitSide;
            if (!fly(speed + SPEED_PROBE, yaw)) break;
            double upDv = (hitUp - up) / SPEED_PROBE, sideDv = (hitSide - side) / SPEED_PROBE;
            if (!fly(speed, yaw + YAW_PROBE)) break;
            double upDyaw = (hitUp - up) / YAW_PROBE, sideDyaw = (hitSide - side) / YAW_PROBE;
            upPerSpeed = upDv;
            sidePerYaw = sideDyaw;

            if (Math.abs(up) < CONVERGED && Math.abs(side) < CONVERGED) {
                converged = true;
                break;
            }
            double det = upDv * sideDyaw - upDyaw * sideDv;
            if (Math.abs(det) < 1e-9) break;
            double dSpeed = (sideDyaw * up - upDyaw * side) / det;
            double dYaw = (upDv * side - sideDv * up) / det;
            speed -= clamp(dSpeed, 50);
            yaw -= clamp(dYaw, 0.3);
            if (speed <= 0) break;
        }

        double maxSpeed = FLYWHEEL_MAX_VELOCITY / FLYWHEEL_TICKS_PER_EXIT_SPEED;
        if (!converged || speed > maxSpeed || upPerSpeed <= 0) {
            lastSpeed = Double.NaN; // start fresh next loop
            // Keep the turret on the target and the flywheel near a sensible speed, ready for when a shot opens up
            double guess = Double.isNaN(speed) || speed <= 0 ? firstGuess(distance) : speed;
            return new Shot(false, Math.min(guess, maxSpeed), Angle.normalizeSigned(directYaw - pose.heading()),
                    turretRate(directYaw, distance, robotVelocity.omega), 0, 0, 0, 0, distance);
        }

        // 2. Most reliable speed: the middle of the range that scores, at this heading
        double halfHeight = HIVE_OPENING_HEIGHT / 2 - BALL_DIAMETER / 2 - SCORING_MARGIN;
        double halfWidth = HIVE_OPENING_WIDTH / 2 - BALL_DIAMETER / 2 - SCORING_MARGIN;
        double low = speedForUp(-halfHeight, speed, yaw, upPerSpeed);
        double high = speedForUp(halfHeight, speed, yaw, upPerSpeed);
        double best = (low + high) / 2;
        double speedTolerance = (high - low) / 2;

        // Re-center sideways at the new speed (it moves a little when the robot is moving across)
        if (fly(best, yaw) && Math.abs(sidePerYaw) > 1e-6) yaw -= hitSide / sidePerYaw;
        if (!fly(best, yaw) || best > maxSpeed) {
            lastSpeed = Double.NaN;
            return new Shot(false, Math.min(speed, maxSpeed), Angle.normalizeSigned(yaw - pose.heading()),
                    turretRate(yaw, distance, robotVelocity.omega), 0, 0, 0, 0, distance);
        }

        lastSpeed = speed; // warm start from the center-hit solution, which is what Newton solves for
        lastYaw = yaw;
        return new Shot(true, best, Angle.normalizeSigned(yaw - pose.heading()),
                turretRate(yaw, distance, robotVelocity.omega), speedTolerance,
                Math.abs(sidePerYaw) > 1e-6 ? halfWidth / Math.abs(sidePerYaw) : 0,
                hitTime, Angle.normalizeSigned(yaw - directYaw), distance);
    }

    /** Turret pivot in field coordinates. */
    public static Pose turretPivot(Pose robotPose) {
        double cos = Math.cos(robotPose.heading());
        double sin = Math.sin(robotPose.heading());
        return new Pose(
                robotPose.x() + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin,
                robotPose.y() + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos,
                robotPose.heading());
    }

    private void setUp(Pose pose, Velocity velocity, Pose target) {
        Pose pivot = turretPivot(pose);
        pivotX = pivot.x();
        pivotY = pivot.y();
        if (SHOOT_ON_THE_MOVE) {
            // Pivot velocity = robot velocity + omega × (pivot - robot center)
            ballVx = velocity.vx - velocity.omega * (pivotY - pose.y());
            ballVy = velocity.vy + velocity.omega * (pivotX - pose.x());
        } else {
            ballVx = 0;
            ballVy = 0;
        }

        centerX = target.x();
        centerY = target.y();
        double outX = Math.cos(target.heading()), outY = Math.sin(target.heading());
        double cosTilt = Math.cos(HIVE_OPENING_TILT), sinTilt = Math.sin(HIVE_OPENING_TILT);
        normalX = outX * cosTilt;
        normalY = outY * cosTilt;
        normalZ = sinTilt;
        upX = -outX * sinTilt;
        upY = -outY * sinTilt;
        upZ = cosTilt;
        sideX = -outY;
        sideY = outX;

        double radius = BALL_DIAMETER / 2 * METERS_PER_INCH;
        k = 0.5 * AIR_DENSITY * Math.PI * radius * radius / BALL_MASS * METERS_PER_INCH;
    }

    /** Vacuum speed for the fixed hood to reach the opening's center from a standstill. */
    private double firstGuess(double distance) {
        double rise = HIVE_OPENING_CENTER_Z - EXIT_HEIGHT;
        double cos = Math.cos(HOOD_ANGLE);
        double denominator = 2 * cos * cos * (distance * Math.tan(HOOD_ANGLE) - rise);
        if (denominator <= 0) return FLYWHEEL_MAX_VELOCITY / FLYWHEEL_TICKS_PER_EXIT_SPEED * 0.8;
        return Math.sqrt(GRAVITY * distance * distance / denominator);
    }

    /** Speed at which the ball crosses the face `up` inches from its center, by secant from a nearby solution. */
    private double speedForUp(double up, double speed, double yaw, double upPerSpeed) {
        double estimate = speed + up / upPerSpeed;
        for (int i = 0; i < 3; i++) {
            if (!fly(estimate, yaw)) break;
            double error = hitUp - up;
            if (Math.abs(error) < CONVERGED) break;
            estimate -= error / upPerSpeed;
        }
        return estimate;
    }

    /** Rate the turret has to turn to keep this heading on a fixed point as the pivot moves and the robot spins. */
    private double turretRate(double yaw, double distance, double omega) {
        return (Math.sin(yaw) * ballVx - Math.cos(yaw) * ballVy) / distance - omega;
    }

    /**
     * Fly a ball launched at this speed and field heading. Returns false if it never crosses the opening's plane
     * heading inward; otherwise hitUp / hitSide say where it crossed (inches from the center) and hitTime when.
     */
    private boolean fly(double speed, double yaw) {
        double cosYaw = Math.cos(yaw), sinYaw = Math.sin(yaw);
        double horizontal = speed * Math.cos(HOOD_ANGLE);
        double x = pivotX + EXIT_RADIUS * cosYaw, y = pivotY + EXIT_RADIUS * sinYaw, z = EXIT_HEIGHT;
        double vx = horizontal * cosYaw + ballVx, vy = horizontal * sinYaw + ballVy, vz = speed * Math.sin(HOOD_ANGLE);
        // Backspin: horizontal axis across the barrel, so the lift starts out straight up
        spinX = sinYaw;
        spinY = -cosYaw;

        double side = planeSide(x, y, z);
        for (double t = 0; t < MAX_FLIGHT; t += STEP) {
            // RK4: position' = velocity, velocity' = accel(velocity)
            accel(vx, vy, vz);
            double k1x = ax, k1y = ay, k1z = az;
            accel(vx + k1x * STEP / 2, vy + k1y * STEP / 2, vz + k1z * STEP / 2);
            double k2x = ax, k2y = ay, k2z = az;
            accel(vx + k2x * STEP / 2, vy + k2y * STEP / 2, vz + k2z * STEP / 2);
            double k3x = ax, k3y = ay, k3z = az;
            accel(vx + k3x * STEP, vy + k3y * STEP, vz + k3z * STEP);
            double k4x = ax, k4y = ay, k4z = az;

            double nx = x + STEP * (vx + STEP / 6 * (k1x + k2x + k3x));
            double ny = y + STEP * (vy + STEP / 6 * (k1y + k2y + k3y));
            double nz = z + STEP * (vz + STEP / 6 * (k1z + k2z + k3z));
            double nvx = vx + STEP / 6 * (k1x + 2 * k2x + 2 * k3x + k4x);
            double nvy = vy + STEP / 6 * (k1y + 2 * k2y + 2 * k3y + k4y);
            double nvz = vz + STEP / 6 * (k1z + 2 * k2z + 2 * k3z + k4z);

            double nextSide = planeSide(nx, ny, nz);
            if (side > 0 && nextSide <= 0) {
                double f = side / (side - nextSide);
                double hx = x + f * (nx - x) - centerX;
                double hy = y + f * (ny - y) - centerY;
                double hz = z + f * (nz - z) - HIVE_OPENING_CENTER_Z;
                double inward = -((vx + f * (nvx - vx)) * normalX + (vy + f * (nvy - vy)) * normalY
                        + (vz + f * (nvz - vz)) * normalZ);
                if (inward <= 0) return false;
                hitUp = hx * upX + hy * upY + hz * upZ;
                hitSide = hx * sideX + hy * sideY;
                hitTime = t + f * STEP;
                return true;
            }
            if (nz < 0) return false;
            x = nx; y = ny; z = nz;
            vx = nvx; vy = nvy; vz = nvz;
            side = nextSide;
        }
        return false;
    }

    /** Distance in front of the opening's plane (positive on the robot's side). */
    private double planeSide(double x, double y, double z) {
        return (x - centerX) * normalX + (y - centerY) * normalY + (z - HIVE_OPENING_CENTER_Z) * normalZ;
    }

    /** Gravity, drag against the velocity, and lift across it from the backspin. Writes ax, ay, az. */
    private void accel(double vx, double vy, double vz) {
        double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double drag = k * DRAG_COEFFICIENT * speed;
        // Lift along spin × velocity, magnitude k·Cl·speed²
        double lx = spinY * vz, ly = -spinX * vz, lz = spinX * vy - spinY * vx;
        double lNorm = Math.sqrt(lx * lx + ly * ly + lz * lz);
        double lift = lNorm > 1e-9 ? k * LIFT_COEFFICIENT * speed * speed / lNorm : 0;
        ax = -drag * vx + lift * lx;
        ay = -drag * vy + lift * ly;
        az = -drag * vz + lift * lz - GRAVITY;
    }

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
