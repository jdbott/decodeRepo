package org.firstinspires.ftc.teamcode.autoshared;

/**
 * Single source of truth for NearAutoByYuvi's tunables and path geometry (BLUE-NATIVE).
 *
 * <p>Both the real opmode {@code autos.NearAutoByYuvi} and any simulator copies
 * read their constants from here, so a pose/timing/geometry edit happens in exactly
 * one place instead of drifting between files.
 *
 * <p><b>Must stay plain Java</b> — no Pedro, Android, or other imports. This file is
 * compiled into the robot build (it lives in TeamCode) <i>and</i> into any plain-Java
 * simulator modules.
 */
public final class YuviNearAutoConfig {
    private YuviNearAutoConfig() {}

    // ===== Fixed shot settings =====
    public static final double SHOT_HOOD_DEG = 48.0;
    public static final double SHOT_FLYWHEEL_RAD = 445.0;

    // ===== Field positions (BLUE-NATIVE) =====
    public static final double START_X = 33.11118657953564;
    public static final double START_Y = 125.85974018244295;
    public static final double START_HEADING_DEG = 180.0;

    public static final double BLUE_TARGET_X = 5.0;
    public static final double TARGET_Y = 139.0;

    // ===== Turret config =====
    public static final double TURRET_CENTER_OFFSET_IN = 1.5;
    public static final double TURRET_MIN_DEG = -180.0;
    public static final double TURRET_MAX_DEG = 180.0;
    public static final double TURRET_OFFSET_DEG = 180.0;
    public static final double SHOT_TIME_SEC = 0.0;   // shot-on-move compensation (set to tuned value)

    // ===== Key poses (BLUE-NATIVE) =====
    // Shot 1
    public static final double SHOT_1_X = 43.0;
    public static final double SHOT_1_Y = 110.0;
    public static final double SHOT_1_HEADING_DEG = 90.0;

    // Shot 2
    public static final double SHOT_2_X = 48.0;
    public static final double SHOT_2_Y = 99.4;
    public static final double SHOT_2_HEADING_DEG = 90.0;

    // Intake 1 geometry
    public static final double INTAKE_1_CTRL_X = 45.0;
    public static final double INTAKE_1_CTRL_Y = 83.0;
    public static final double INTAKE_1_END_X = 18.3;
    public static final double INTAKE_1_END_Y = 82.2;
    public static final double INTAKE_1_HEADING_DEG = 90.0;

    // Shot 3
    public static final double SHOT_3_X = 54.0;
    public static final double SHOT_3_Y = 93.0;
    public static final double SHOT_3_HEADING_DEG = 90.0;

    // Intake 2 geometry
    public static final double INTAKE_2_CTRL_X = 33.0;
    public static final double INTAKE_2_CTRL_Y = 60.0;
    public static final double INTAKE_2_END_X = 18.0;
    public static final double INTAKE_2_END_Y = 59.0;
    public static final double INTAKE_2_HEADING_DEG = 90.0;

    // Shot 4
    public static final double SHOT_4_X = 30.0;
    public static final double SHOT_4_Y = 75.0;
    public static final double SHOT_4_HEADING_DEG = 90.0;

    // Intake 3 / 4 (shared location)
    public static final double INTAKE_3_X = 8.0;
    public static final double INTAKE_3_Y = 67.0;
    public static final double INTAKE_3_HEADING_DEG = 90.0;

    // Shot 5
    public static final double SHOT_5_X = 63.0;
    public static final double SHOT_5_Y = 82.0;
    public static final double SHOT_5_HEADING_DEG = 90.0;

    // Park
    public static final double PARK_X = 56.0;
    public static final double PARK_Y = 129.0;
    public static final double PARK_HEADING_DEG = 270.0;

    // ===== Timing (seconds) =====
    public static final double FLYWHEEL_SPIN_UP_SEC = 1.2;
    public static final double FEED_START_DELAY_SEC = 0.08;
    public static final double FEED_TOTAL_SEC = 0.75;
    public static final double INTAKE_DWELL_SEC = 0.4;      // short dwell for intake 1 & 2
    public static final double INTAKE_3_DWELL_SEC = 2.25;   // dwell at (8, 67) first visit
    public static final double INTAKE_4_DWELL_SEC = 2.25;   // dwell at (8, 67) second visit

    // ===== Drive params =====
    public static final double DRIVE_MAX_POWER = 1.0;
    public static final double INTAKE_THROTTLED_POWER = 0.6;
}