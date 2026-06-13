package org.firstinspires.ftc.teamcode.autosim.model;

import org.firstinspires.ftc.teamcode.autosim.geom.Pt;

import java.util.List;

/** One path segment: its control points (source of truth) and the sampled polyline. */
public final class PathSpec {
    public String id;
    public String kind;            // "LINE" | "CURVE"
    public List<Pt> controlPoints;
    public List<Pt> polyline;
    public double headingDeg;
    public long tStartMs = -1;     // when this segment is driven (set in Phase 2)
    public long tEndMs = -1;

    public PathSpec(String id, String kind, List<Pt> controlPoints, List<Pt> polyline, double headingDeg) {
        this.id = id;
        this.kind = kind;
        this.controlPoints = controlPoints;
        this.polyline = polyline;
        this.headingDeg = headingDeg;
    }
}
