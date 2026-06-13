package org.firstinspires.ftc.teamcode.autosim.geom;

/** A field-space pose: position in inches, heading in degrees (CCW, 0 = +X). */
public final class Pose2d {
    public final double x, y, headingDeg;

    public Pose2d(double x, double y, double headingDeg) {
        this.x = x;
        this.y = y;
        this.headingDeg = headingDeg;
    }
}
