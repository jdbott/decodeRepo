package org.firstinspires.ftc.teamcode.autosim.autos;

import org.firstinspires.ftc.teamcode.autosim.geom.Bezier;
import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;
import org.firstinspires.ftc.teamcode.autosim.geom.Pt;
import org.firstinspires.ftc.teamcode.autosim.model.Frame;
import org.firstinspires.ftc.teamcode.autosim.model.PathSpec;
import org.firstinspires.ftc.teamcode.autosim.model.SimTrace;

import java.util.Arrays;
import java.util.List;

/**
 * Simulatable copy of V3FarAuto's path geometry (BLUE-NATIVE).
 *
 * <p>Phase 1 only reproduces the path-building portion of the auto so the viewer can
 * render the route statically. The constants below are copied verbatim from
 * {@code V3FarAuto.java}; they are the acknowledged "FSM-copy" duplication (see
 * AUTOSIM_DESIGN.md §5). Phase 2 adds the SimClock/SimFollower and the two FSMs.
 *
 * <p>The original V3FarAuto.java is never imported or modified.
 */
public final class V3FarAutoSim {
    // ===== Field positions (BLUE-NATIVE) — copied from V3FarAuto =====
    static final double START_X = 64.1, START_Y = 6.75, START_HEADING_DEG = 180.0;
    static final double BLUE_TARGET_X = 5.0, TARGET_Y = 139.0;
    static final double SHOOT2_Y_OFFSET = 5.5;
    static final double LAST_LINE_X = 12.5, LAST_LINE_Y = 38.0;
    static final double INTAKE_1_X = 15.0, INTAKE_1_Y = 9.5;
    static final double BACKUP_DISTANCE = 10.0, ALT_CYCLE_Y_OFFSET = 19.0;
    static final double CURVE_CTRL_X = 45.0, CURVE_CTRL_Y = 38.0;   // toLastLine control point
    static final double RETREAT_DX = -23.0;                          // blue-native retreat

    // Initial mechanism state (for the Phase 1 start frame).
    static final double INIT_TURRET_ANGLE_DEG = 113.0;
    static final double FIXED_HOOD_DEG = 53.5;
    static final double FIXED_FLYWHEEL_RAD = 445;

    static final int CURVE_SEGMENTS = 40;

    private V3FarAutoSim() {}

    public static SimTrace build() {
        SimTrace t = new SimTrace();
        t.meta.autoName = "V3FarAuto";
        t.meta.startPose = new Pose2d(START_X, START_Y, START_HEADING_DEG);
        t.field.goal = new Pt(BLUE_TARGET_X, TARGET_Y);
        t.field.backgroundImage = "fields/decode.svg";

        Pt shoot1 = new Pt(START_X, START_Y);
        Pt shoot2 = new Pt(START_X, START_Y + SHOOT2_Y_OFFSET);
        Pt lastLine = new Pt(LAST_LINE_X, LAST_LINE_Y);

        // ----- Base paths (buildBasePaths) -----
        t.paths.add(curve("toLastLine",
                Arrays.asList(shoot1, new Pt(CURVE_CTRL_X, CURVE_CTRL_Y), lastLine), 180));
        t.paths.add(line("fromLastLineToShoot2", lastLine, shoot2, 180));

        // ----- Cycle paths (buildCyclePathsForCurrentCycle): cycle 0 = low, 1 = high -----
        addCycle(t, 0, shoot2);
        addCycle(t, 1, shoot2);

        // ----- Retreat (final park) -----
        t.paths.add(line("retreat", shoot2, new Pt(START_X + RETREAT_DX, START_Y + SHOOT2_Y_OFFSET), 180));

        // ----- Phase 1 start frame so the viewer can place the robot -----
        Frame f0 = new Frame();
        f0.tMs = 0;
        f0.pose = t.meta.startPose;
        f0.autoState = "WAIT_INITIAL_DELAY";
        f0.feedState = "IDLE";
        f0.followerBusy = false;
        f0.flywheelTargetRadS = FIXED_FLYWHEEL_RAD;
        f0.flywheelActualRadS = 0;
        f0.turretDeg = INIT_TURRET_ANGLE_DEG;
        f0.hoodDeg = FIXED_HOOD_DEG;
        f0.intakePower = 0;
        t.frames.add(f0);

        return t;
    }

    private static void addCycle(SimTrace t, int cycle, Pt shoot2) {
        double yOff = (cycle % 2 == 1) ? ALT_CYCLE_Y_OFFSET : 0.0;
        String tag = (cycle % 2 == 1) ? "high" : "low";
        Pt intake = new Pt(INTAKE_1_X, INTAKE_1_Y + yOff);
        Pt backed = new Pt(INTAKE_1_X + BACKUP_DISTANCE, INTAKE_1_Y + yOff);
        t.paths.add(line("toIntake1 (" + tag + ")", shoot2, intake, 200));
        t.paths.add(line("backup (" + tag + ")", intake, backed, 180));
        t.paths.add(line("backToShoot2 (" + tag + ")", backed, shoot2, 180));
    }

    private static PathSpec line(String id, Pt a, Pt b, double headingDeg) {
        List<Pt> cp = Arrays.asList(a, b);
        return new PathSpec(id, "LINE", cp, Bezier.sample(cp, 1), headingDeg);
    }

    private static PathSpec curve(String id, List<Pt> control, double headingDeg) {
        return new PathSpec(id, "CURVE", control, Bezier.sample(control, CURVE_SEGMENTS), headingDeg);
    }
}
