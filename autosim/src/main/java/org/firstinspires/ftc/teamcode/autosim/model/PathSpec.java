package org.firstinspires.ftc.teamcode.autosim.model;

import org.firstinspires.ftc.teamcode.autosim.geom.Pt;

import java.util.List;

/** One path segment: its control points (source of truth) and the sampled polyline. */
public final class PathSpec {
    public String id;
    public String kind;            // "LINE" | "CURVE"
    public List<Pt> controlPoints;
    public List<Pt> polyline;
    public double headingDeg;      // nominal heading; the constant target when mode == CONSTANT
    public long tStartMs = -1;     // when this segment is driven (set in Phase 2)
    public long tEndMs = -1;

    // How heading is driven along this path (SimFollower honors this; not serialized — heading is
    // baked into the per-frame poses). Defaults to CONSTANT to match the original behavior.
    public HeadingMode headingMode = HeadingMode.CONSTANT;
    public double headingStartDeg;     // LINEAR only
    public double headingEndDeg;       // LINEAR only
    public double headingEndT = 1.0;   // LINEAR only: path fraction over which the slew completes

    // Mid-path mode switch (Pedro paths sometimes re-call setXxxHeadingInterpolation() partway
    // through, e.g. once getCurrentTValue() crosses a threshold). headingSwitchT < 0 means no
    // switch; once the path fraction t exceeds headingSwitchT, headingModeAfter/headingDegAfter
    // take over from headingMode/headingDeg.
    public double headingSwitchT = -1.0;
    public HeadingMode headingModeAfter = HeadingMode.CONSTANT;
    public double headingDegAfter;

    public PathSpec(String id, String kind, List<Pt> controlPoints, List<Pt> polyline, double headingDeg) {
        this.id = id;
        this.kind = kind;
        this.controlPoints = controlPoints;
        this.polyline = polyline;
        this.headingDeg = headingDeg;
    }

    /** Face along the path tangent (Pedro's {@code setTangentHeadingInterpolation}). */
    public PathSpec tangent() { headingMode = HeadingMode.TANGENT; return this; }

    /** Face tangent + 180° (Pedro's {@code reverseHeadingInterpolation}). */
    public PathSpec reverseTangent() { headingMode = HeadingMode.REVERSE_TANGENT; return this; }

    /** Slew start°→end° over the first {@code endT} of the path (Pedro's linear interpolation). */
    public PathSpec linear(double startDeg, double endDeg, double endT) {
        headingMode = HeadingMode.LINEAR;
        headingStartDeg = startDeg;
        headingEndDeg = endDeg;
        headingEndT = endT;
        return this;
    }

    /**
     * Once the path is more than {@code afterT} complete (by arc length), switch to a constant
     * heading of {@code headingDeg} — mirrors a real auto re-calling
     * {@code setConstantHeadingInterpolation(...)} after {@code getCurrentTValue() > afterT}.
     */
    public PathSpec thenConstant(double afterT, double headingDeg) {
        headingSwitchT = afterT;
        headingModeAfter = HeadingMode.CONSTANT;
        headingDegAfter = headingDeg;
        return this;
    }
}
