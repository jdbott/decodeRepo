package org.firstinspires.ftc.teamcode.autoshared;

/**
 * Single source of truth for V3FarAuto's tunables and path geometry (BLUE-NATIVE).
 *
 * <p>Both the real opmode {@code autos.V3FarAuto} and the simulator copy
 * {@code autosim.autos.V3FarAutoSim} read their constants from here, so a pose/timing/geometry
 * edit happens in exactly one place instead of drifting between the two files.
 *
 * <p><b>Must stay plain Java</b> — no Pedro, Android, or other imports. This file is compiled
 * into the robot build (it lives in TeamCode) <i>and</i> into the plain-Java {@code autosim}
 * module (which pulls this package in via a source dir). Each side adapts these neutral numbers
 * into its own types (the real auto builds Pedro {@code Pose}/{@code BezierCurve}; the sim builds
 * its polylines).
 */
public final class V3FarAutoConfig {
    private V3FarAutoConfig() {}

    // ===== Fixed shot settings =====
    public static final double FIXED_HOOD_DEG = 53.5;
    public static final double FIXED_FLYWHEEL_RAD = 445;

    // ===== Field positions (BLUE-NATIVE) =====
    public static final double START_X = 64.1;
    public static final double START_Y = 6.75;
    public static final double START_HEADING_DEG = 180.0;

    public static final double BLUE_TARGET_X = 5.0;
    public static final double TARGET_Y = 139.0;

    public static final double SHOOT2_Y_OFFSET = 5.5;

    public static final double LAST_LINE_X = 12.5;
    public static final double LAST_LINE_Y = 38.0;

    public static final double INTAKE_1_X = 15.0;
    public static final double INTAKE_1_Y = 9.5;

    public static final double BACKUP_DISTANCE = 10.0;
    public static final double ALT_CYCLE_Y_OFFSET = 19.0;

    // Control point of the start->lastLine curve.
    public static final double CURVE_CTRL_X = 45.0;
    public static final double CURVE_CTRL_Y = 38.0;

    // Final retreat distance (magnitude; blue applies -X, red +X).
    public static final double RETREAT_DX = 23.0;

    // ===== Constant-interpolation headings (deg, BLUE-NATIVE) =====
    public static final double DRIVE_HEADING_DEG = 180.0;
    public static final double INTAKE_HEADING_DEG = 200.0;

    // ===== Fixed turret aim commands (BLUE-NATIVE) =====
    public static final double INIT_TURRET_ANGLE_DEG = 113.0;
    public static final double RUN_TURRET_ANGLE_DEG = 113.0;

    // ===== Timing (seconds) =====
    public static final double FIRST_SHOT_DELAY_SEC = 3;
    public static final double FEED_START_DELAY_SEC = 0.10;
    public static final double FEED_TOTAL_TIME_SEC = 1.00;
    public static final double REVERSE_TIME_SEC = 0.25;
}
