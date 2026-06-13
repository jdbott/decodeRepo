package org.firstinspires.ftc.teamcode.autosim.sim;

/** Recording stub mirroring Turret's surface (commanded angle tracked instantly). */
public final class SimTurret {
    private double angle = 0;

    public void setAngle(double a) { angle = a; }
    public void update() {}
    public double getCurrentAngle() { return angle; }
}
