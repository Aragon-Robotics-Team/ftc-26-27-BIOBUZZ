/**
 * What a mecanum drivetrain can do, from its motors and wheels.
 *
 * Each wheel's motor has a voltage budget: part goes to spinning (back-EMF, in proportion to wheel speed / free speed)
 * and the rest to pushing (force in proportion to stall force). With four mecanum wheels, the worst wheel's share of
 * its budget is the sum of what forward, sideways and turning each ask of it, so the robot can move as
 *
 *   |vx|/vForward + |vy|/vStrafe + |ω|/ωMax  +  |ax|/aForward + |ay|/aStrafe + |α|/αMax  ≤  battery / spec voltage
 *
 * (x = robot forward, y = robot left). Separately, no wheel can push harder than grip allows:
 *
 *   |ax| + |ay| + |α|·I/(m·R)  ≤  grip · g
 */
import type { ModelConfig, RobotConfig } from './project.ts';

/** Standard gravity, in/s². */
const G = 386.09;
const IN = 0.0254; // m per inch

export interface Drivetrain {
  /** Top speeds, in/s and rad/s. */
  vForward: number;
  vStrafe: number;
  omegaMax: number;
  /** Accelerations the motors could give from a standstill, in/s² and rad/s². */
  aForward: number;
  aStrafe: number;
  alphaMax: number;
  /** Grip limit, in/s², and how much of it a unit of angular acceleration uses (in/s² per rad/s²). */
  traction: number;
  spinShare: number;
  coastForward: number;
  coastStrafe: number;
}

export function drivetrain(model: ModelConfig, robot: RobotConfig): Drivetrain {
  const wheelFree = ((model.motorRpm / model.reduction) / 60) * Math.PI * model.wheelDiameter; // in/s
  const vForward = wheelFree * model.driveEfficiency;
  const vStrafe = vForward * model.strafeEfficiency;
  // Mecanum: a wheel's rim speed from turning is ω·(half wheelbase + half track width).
  const lever = (model.wheelbase + model.trackWidth) / 2; // in
  const omegaMax = vForward / lever;
  // Stall force per wheel, N, after the same losses as speed.
  const stallForce = ((model.motorStallTorque * model.reduction) / ((model.wheelDiameter / 2) * IN)) * model.driveEfficiency;
  const aForward = ((4 * stallForce) / model.mass) / IN; // in/s²
  const aStrafe = aForward * model.strafeEfficiency;
  // Robot as a uniform box of its footprint: I = m (L² + W²) / 12.
  const inertia = (model.mass * ((robot.length * IN) ** 2 + (robot.width * IN) ** 2)) / 12; // kg·m²
  const alphaMax = (4 * stallForce * lever * IN) / inertia; // rad/s²
  return {
    vForward,
    vStrafe,
    omegaMax,
    aForward,
    aStrafe,
    alphaMax,
    traction: model.grip * G,
    spinShare: inertia / (model.mass * lever * IN) / IN, // (kg·m²)/(kg·m) = m → in
    coastForward: model.coastForward,
    coastStrafe: model.coastStrafe,
  };
}
