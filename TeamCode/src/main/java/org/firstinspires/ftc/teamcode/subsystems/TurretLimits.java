package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Where the turret is sent for a wanted angle, given its soft limits. In range: the wanted angle. Out of range (the
 * blind zone behind the robot): a limit, chosen once as the target leaves the range (the one nearest it, the edge it
 * left through) and kept while the target stays out. Choosing the nearest limit every loop instead, a target wobbling
 * across straight back (±180°) swings the turret all the way between the limits and back. No hardware, so it's unit
 * tested.
 */
final class TurretLimits {
    private TurretLimits() {}

    /**
     * @param wanted         where the shot wants the turret, radians in (-π, π]
     * @param previousTarget what this returned last loop
     * @param wasClipped     whether wanted was out of range last loop
     * @return the angle to drive the turret to, within [min, max]
     */
    static double target(double wanted, double previousTarget, boolean wasClipped, double min, double max) {
        if (wanted >= min && wanted <= max) return wanted;
        // Already parked on a limit: stay there. (Checked against the limits, not just wasClipped, so a limit changed
        // from Panels mid-match can't leave the turret parked off one.)
        if (wasClipped && (previousTarget == min || previousTarget == max)) return previousTarget;
        // Just left the range: the limit nearest the target
        return wanted > 0 ? max : min;
    }
}
