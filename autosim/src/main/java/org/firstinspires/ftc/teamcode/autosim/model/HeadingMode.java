package org.firstinspires.ftc.teamcode.autosim.model;

/**
 * How a path's heading is driven as the robot follows it — mirrors Pedro Pathing's interpolation
 * modes so the sim can show the real turning behavior instead of a constant snap.
 *
 * <ul>
 *   <li>{@code CONSTANT} — hold a fixed heading ({@code setConstantHeadingInterpolation}).</li>
 *   <li>{@code TANGENT} — face along the path tangent ({@code setTangentHeadingInterpolation}).</li>
 *   <li>{@code REVERSE_TANGENT} — face tangent + 180° ({@code reverseHeadingInterpolation}).</li>
 *   <li>{@code LINEAR} — slew start°→end° over the first {@code headingEndT} of the path
 *       ({@code setLinearHeadingInterpolation}).</li>
 * </ul>
 */
public enum HeadingMode { CONSTANT, TANGENT, REVERSE_TANGENT, LINEAR }
