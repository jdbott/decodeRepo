package org.firstinspires.ftc.teamcode.autosim.model;

/**
 * The robot's drawn footprint, relative to the tracked (odometry / wheelbase) point that
 * the pose refers to.
 *
 * <ul>
 *   <li>{@code lengthIn} — fore-aft extent, along the heading axis.</li>
 *   <li>{@code widthIn} — left-right extent.</li>
 *   <li>{@code wheelbaseOffsetIn} — how far <b>forward</b> the body's geometric center sits
 *       from the tracked point. Positive pushes the body (e.g. a front intake) ahead of the
 *       wheelbase, so the tracked point ends up behind the body center.</li>
 * </ul>
 *
 * These are display/config defaults baked into the trace; the viewer lets you adjust them live.
 */
public final class RobotSpec {
    public double lengthIn = 18;
    public double widthIn = 18;
    public double wheelbaseOffsetIn = 0;
}
