package org.firstinspires.ftc.teamcode.autoshared;

/**
 * Single source of truth for V3Auto's tunables, path geometry, and the distance→shot lookup table
 * (BLUE-NATIVE). Read by both the real opmode {@code autos.V3Auto} and the simulator copy
 * {@code autosim.autos.V3AutoSim}.
 *
 * <p><b>Must stay plain Java</b> — no Pedro/Android imports (see {@link V3FarAutoConfig}).
 */
public final class V3AutoConfig {
    private V3AutoConfig() {}

    // ===== Start pose (BLUE-NATIVE) =====
    public static final double START_X = 20.75;
    public static final double START_Y = 128.1;
    public static final double START_HEADING_DEG = -39.38;

    // First-shot pose is projected off the start pose along its heading.
    public static final double FIRST_SHOT_FWD_IN = 35.0;   // along cos(startHeading)
    public static final double FIRST_SHOT_SIDE_IN = 55.0;  // along sin(startHeading)

    // ===== First-shot fixed setup =====
    public static final double FIRST_SHOT_HOOD_DEG = 37;
    public static final double FIRST_SHOT_FLYWHEEL_RAD = 310;

    // ===== Goal (BLUE-NATIVE) =====
    public static final double BLUE_TARGET_X = 5.0;
    public static final double TARGET_Y = 139.0;

    // ===== Turret =====
    public static final double TURRET_CENTER_OFFSET_IN = 1.5;
    public static final double TURRET_MIN_DEG = -180.0;
    public static final double TURRET_MAX_DEG = 180.0;
    public static final double TURRET_OFFSET_DEG = 180.0;
    public static final double INIT_TURRET_ANGLE_DEG = 90.0;

    // Extra intake nudge at the gate.
    public static final double EXTRA_GATE_INTAKE_Y_IN = 4.5;

    public static final double PREDICTED_DISTANCE_ALPHA = 0.45;
    public static final double SHOT_TIME_SEC = 0.77;

    // ===== Path control points (BLUE-NATIVE) =====
    // Middle line cycle.
    public static final double LINE2_C1X = 48.0, LINE2_C1Y = 67.0;
    public static final double LINE2_C2X = 40.0, LINE2_C2Y = 63.0;
    public static final double LINE2_ENDX = 12.5, LINE2_ENDY = 62.0;
    public static final double BTS_STARTX = 11.0, BTS_STARTY = 65.0;
    public static final double BTS_C1X = 30.0, BTS_C1Y = 65.0;

    // Gate approach + intake.
    public static final double GATE_C1X = 50.0, GATE_C1Y = 66.326;
    public static final double GATE_ENDX = 15.0, GATE_ENDY = 67.5;
    public static final double GI_STARTX = 16.0, GI_STARTY = 69.5;
    public static final double GI_ENDX = 11.0, GI_ENDY = 59.0;
    public static final double RG1_C1X = 42.0, RG1_C1Y = 59.0;   // return-from-gate cycle 1
    public static final double RG2_C1X = 42.0, RG2_C1Y = 62.0;   // return-from-gate cycle 2

    // Higher-Y closer line (fourth pickup).
    public static final double FP_C1X = 40.0, FP_C1Y = 84.0;
    public static final double FP_ENDX = 20.0, FP_ENDY = 84.0;

    // Last line cycle.
    public static final double LL_C1X = 48.0, LL_C1Y = 43.0;
    public static final double LL_C2X = 40.0, LL_C2Y = 36.0;
    public static final double LL_ENDX = 12.5, LL_ENDY = 35.0;
    public static final double BFLL_STARTX = 11.0, BFLL_STARTY = 41.0;
    public static final double BFLL_C1X = 30.0, BFLL_C1Y = 41.0;
    // backToShootFromLastLine's end pose is firstShotPose offset by this much (BLUE-NATIVE).
    public static final double LAST_RETURN_END_DX = 10.0;
    public static final double LAST_RETURN_END_DY = 18.0;

    // ===== Constant/linear/thenConstant interpolation headings (deg, BLUE-NATIVE) =====
    public static final double LINE_HEADING_DEG = 180.0;
    public static final double GATE_OPEN_HEADING_START_DEG = 235.0;
    public static final double GATE_OPEN_HEADING_END_DEG = 170.0;
    public static final double GATE_OPEN_HEADING_T = 0.9;
    public static final double GATE_INTAKE_HEADING_DEG = 135.0;
    public static final double BACK_TO_SHOOT_LAST_HEADING_DEG = -90.0;

    // ===== Timing (seconds) =====
    public static final double FEED_START_DELAY_SEC = 0.1;
    public static final double FEED_TOTAL_TIME_SEC = 1.0;
    public static final double REVERSE_TIME_SEC = 0.25;

    public static final double WAIT_AT_GATE_SEC = 0.1;
    public static final double GATE_INTAKE_SEC = 1.5;
    public static final double WAIT_AT_GATE_AGAIN_SEC = 0.25;
    public static final double GATE_INTAKE_AGAIN_SEC = 1.3;
    public static final double EXTRA_MOVE_TIMEOUT_SEC = 1.0;

    // ===== Gating thresholds (parametric path progress) =====
    public static final double FIRST_SHOT_FEED_TVALUE = 0.9;
    public static final double GATE_INTAKE_ON_TVALUE = 0.5;
    public static final double LINE2_SLOW_TVALUE = 0.35;
    public static final double LINE2_SLOW_POWER = 0.8;
    public static final double LAST_LINE_SLOW_TVALUE = 0.2;
    public static final double LAST_LINE_SLOW_POWER = 0.8;
    public static final double BACK_TO_SHOOT_LAST_SWITCH_TVALUE = 0.8;

    // ===== Distance → {hoodDeg, flywheelRad} lookup (with the source's +5 / +15 tweaks baked in) =====
    public static final double[][] SHOT_TABLE = {
            {37.0, 30.0, 267.0},
            {43.0, 30.0, 267.0},
            {50.0, 37.0, 277.0 + 5},
            {57.0, 37.0, 282.0 + 5},
            {63.5, 37.0, 292.0 + 5},
            {71.0, 39.0, 307.0 + 5},
            {77.0, 40.0, 312.0 + 5},
            {82.0, 42.0, 327.0 + 5},
            {88.0, 43.0, 332.0 + 5},
            {93.0, 44.0, 347.0 + 5},
            {99.0, 46.0, 364.0 + 5},
            {104.0, 47.0, 374.0 + 5},
            {110.0, 48.0, 389.0 + 5},
            {122.0, 53.0, 409.5 + 5}
    };
}
