/** Reads the speed numbers out of the Foresight Tuner's printed ForesightConfig block. */
import type { ModelConfig } from '../core/project.ts';

const KEYS: [string, keyof ModelConfig][] = [
  ['maxAchievableForwardVelocity', 'vForward'],
  ['maxAchievableStrafeVelocity', 'vStrafe'],
  ['naturalForwardDeceleration', 'dForward'],
  ['naturalStrafeDeceleration', 'dStrafe'],
];

export function parseForesight(text: string): { values: Partial<ModelConfig>; found: string[] } {
  const values: Partial<ModelConfig> = {};
  const found: string[] = [];
  for (const [name, key] of KEYS) {
    const m = new RegExp(`${name}\\s*(?:\\.set\\(|[:=]\\s*)\\s*(-?[\\d.eE+-]+)`).exec(text);
    if (!m) continue;
    const v = Math.abs(Number(m[1]));
    if (!Number.isFinite(v) || v === 0) continue;
    (values as Record<string, number>)[key] = v;
    found.push(name);
  }
  if (!found.length) throw new Error('No Foresight numbers found. Paste the block with c.maxAchievableForwardVelocity.set(…) and friends.');
  return { values, found };
}
