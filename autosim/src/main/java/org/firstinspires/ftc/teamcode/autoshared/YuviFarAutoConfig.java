package org.firstinspires.ftc.teamcode.autoshared;

/**
 * Single source of truth for V3FarAutoByYuvi's tunables and path geometry (BLUE-NATIVE).
 *
 * <p>Both the real opmode {@code autos.V3FarAutoByYuvi} and any simulator copies
 * read their constants from here, so a pose/timing/geometry edit happens in exactly
 * one place instead of drifting between files.
 *
 * <p><b>Must stay plain Java</b> — no Pedro, Android, or other imports. This file is
 * compiled into the robot build (it lives in TeamCode) <i>and</i> into any plain-Java
 * simulator modules.
 */
public final class YuviFarAutoConfig {
    private YuviFarAutoConfig() {}

    // ===== Fixed shot settings =====
    // Preload shot (first shot from start pose)
    public static final double PRELOAD_HOOD_DEG = 55.5;
    public static final double PRELOAD_FLYWHEEL_RAD = 480.0;

    // Cycle shots (used after intake 1, 2, 3)
    public static final double CYCLE_HOOD_DEG = 40.0;
    public static final double CYCLE_FLYWHEEL_RAD = 317.0;

    // ===== Field positions (BLUE-NATIVE) =====
    public static final double START_X = 56.0;
    public static final double START_Y = 8.0;
    public static final double START_HEADING_DEG = 90.0;

    public static final double BLUE_TARGET_X = 5.0;
    public static final double TARGET_Y = 139.0;

    // ===== Turret config =====
    public static final double TURRET_CENTER_OFFSET_IN = 1.5;
    public static final double TURRET_MIN_DEG = -180.0;
    public static final double TURRET_MAX_DEG = 180.0;
    public static final double TURRET_OFFSET_DEG = 180.0;

    // ===== Key poses (BLUE-NATIVE) =====
    // Intake 1
    public static final double INTAKE_1_START_X = 12.520;
    public static final double INTAKE_1_START_Y = 16.562;
    public static final double INTAKE_1_START_HEADING_DEG = 180.0;
    public static final double INTAKE_1_END_X = 15.343;
    public static final double INTAKE_1_END_Y = 4.837;
    public static final double INTAKE_1_END_HEADING_DEG = 180.0;

    // Shoot 1
    public static final double SHOOT_1_X = 70.646;
    public static final double SHOOT_1_Y = 20.581;
    public static final double SHOOT_1_HEADING_DEG = 180.0;

    // Intake 2
    public static final double INTAKE_2_START_X = 33.219;
    public static final double INTAKE_2_START_Y = 35.214;
    public static final double INTAKE_2_START_HEADING_DEG = 180.0;
    public static final double INTAKE_2_END_X = 18.290;
    public static final double INTAKE_2_END_Y = 35.059;
    public static final double INTAKE_2_END_HEADING_DEG = 180.0;

    // Shoot 2
    public static final double SHOOT_2_X = 69.914;
    public static final double SHOOT_2_Y = 75.762;
    public static final double SHOOT_2_HEADING_DEG = 180.0;

    // Intake 3
    public static final double INTAKE_3_START_X = 32.187;
    public static final double INTAKE_3_START_Y = 59.918;
    public static final double INTAKE_3_START_HEADING_DEG = 180.0;
    public static final double INTAKE_3_END_X = 16.285;
    public static final double INTAKE_3_END_Y = 59.078;
    public static final double INTAKE_3_END_HEADING_DEG = 180.0;

    // Shoot 3
    public static final double SHOOT_3_X = 68.186;
    public static final double SHOOT_3_Y = 78.109;
    public static final double SHOOT_3_HEADING_DEG = 180.0;

    // End
    public static final double END_X = 68.963;
    public static final double END_Y = 16.829;
    public static final double END_HEADING_DEG = 180.0;

    // ===== Path control points (BLUE-NATIVE) =====
    // Start -> Intake 1 zone intermediate point
    public static final double START_TO_INTAKE_1_CTRL_X = 17.876;
    public static final double START_TO_INTAKE_1_CTRL_Y = 18.959;
    public static final double START_TO_INTAKE_1_CTRL_HEADING_DEG = 180.0;

    // ===== Constant-interpolation headings (deg, BLUE-NATIVE) =====
    public static final double DRIVE_HEADING_DEG = 180.0;

    // ===== Fixed turret aim commands (BLUE-NATIVE) =====
    public static final double INIT_TURRET_ANGLE_DEG = 113.0;

    // ===== Timing (seconds) =====
    public static final double FIRST_SHOT_DELAY_SEC = 2.50;
    public static final double FEED_START_DELAY_SEC = 0.10;
    public static final double FEED_TOTAL_TIME_SEC = 0.93;
    public static final double REVERSE_TIME_SEC = 0.25;
    public static final double FLYWHEEL_PREP_SEC = 1.4;
    public static final double INTAKE_DURATION_SEC = 1.25;

    // ===== Distance thresholds (inches) =====
    public static final double INTAKE_ZONE_THRESHOLD_IN = 3.0;
    public static final double FLYWHEEL_PREP_DIST = 22.0;
    public static final double INTAKE_START_DIST = 30.0;
}