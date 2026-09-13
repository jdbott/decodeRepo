package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.config.Modifier;

/**
 * Per-path Foresight overrides — the Pedro 3 replacement for Pedro 2's {@code follower.setMaxPower()}
 * and {@code path.setBrakingStrength()}. Attach with {@code path.with(...)}: the follower applies the
 * override only while that path segment is being followed, then restores the value in {@link Constants}.
 *
 * <pre>
 * curve(a, c, b).tangent().with(PathModifiers.maxSpeed(0.4))
 * // Slow down partway: split the route and put the modifier on the second half
 * path(line(a, b), line(b, c).with(PathModifiers.maxSpeed(0.4)))
 * </pre>
 */
public final class PathModifiers {

    private PathModifiers() {}

    /** Caps speed to a fraction (0–1] of the robot's max achievable velocity. Replaces setMaxPower. */
    public static Modifier maxSpeed(double fraction) {
        return Constants.foresightConfig.maxPathSpeed.at(fraction);
    }

    /** Caps speed to an absolute velocity in in/s. */
    public static Modifier maxVelocity(double inchesPerSec) {
        return Constants.foresightConfig.maxVelocityConstraint.at(inchesPerSec);
    }

    /**
     * Slows down at a fraction (0–1] of the robot's natural deceleration, so it starts slowing earlier
     * and stops more gently. Replaces setBrakingStrength.
     */
    public static Modifier softBraking(double scale) {
        return Constants.foresightConfig.maxDecelerationScale.at(scale);
    }

    /** Above 1 brakes later and allows overshoot (e.g. into a wall); below 1 brakes earlier. */
    public static Modifier brakeAggression(double aggression) {
        return Constants.foresightConfig.brakeAggression.at(aggression);
    }

    /** Skips braking at the end of this path so speed carries into whatever is followed next. */
    public static Modifier coastThrough() {
        return Constants.foresightConfig.brakeAtEnd.at(false);
    }
}
