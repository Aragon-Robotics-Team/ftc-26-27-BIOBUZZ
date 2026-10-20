/** Reads the Foresight Tuner's printed ForesightConfig block and turns it into drivetrain numbers. */
import type { ModelConfig } from '../core/project.ts';

const KEYS = {
  forward: 'maxAchievableForwardVelocity',
  strafe: 'maxAchievableStrafeVelocity',
  coastForward: 'naturalForwardDeceleration',
  coastStrafe: 'naturalStrafeDeceleration',
} as const;

export type ForesightNumbers = Partial<Record<keyof typeof KEYS, number>>;

export function parseForesight(text: string): ForesightNumbers {
  const out: ForesightNumbers = {};
  for (const [key, name] of Object.entries(KEYS) as [keyof typeof KEYS, string][]) {
    const m = new RegExp(`${name}\\s*(?:\\.set\\(|[:=]\\s*)\\s*(-?[\\d.eE+-]+)`).exec(text);
    const v = m ? Math.abs(Number(m[1])) : NaN;
    if (Number.isFinite(v) && v > 0) out[key] = v;
  }
  if (!Object.keys(out).length) throw new Error('No Foresight numbers found. Paste the block with c.maxAchievableForwardVelocity.set(…) and friends.');
  return out;
}

/**
 * Measured top speeds become the drivetrain's efficiencies (measured ÷ what the motors and wheels could do), and the
 * natural decelerations its coasting. The motor, wheels and mass stay as set.
 */
export function applyForesight(model: ModelConfig, n: ForesightNumbers): ModelConfig {
  const wheelFree = ((model.motorRpm / model.reduction) / 60) * Math.PI * model.wheelDiameter;
  const next = { ...model };
  if (n.forward) next.driveEfficiency = n.forward / wheelFree;
  if (n.strafe) next.strafeEfficiency = n.strafe / (n.forward ?? wheelFree * next.driveEfficiency);
  if (n.coastForward) next.coastForward = n.coastForward;
  if (n.coastStrafe) next.coastStrafe = n.coastStrafe;
  return next;
}
