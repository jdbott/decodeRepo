package org.firstinspires.ftc.teamcode.autosim.sim;

/** A virtual clock the sim advances by a fixed dt each tick (replaces wall time). */
public final class SimClock {
    private double tSec = 0;

    public void advance(double dtSec) { tSec += dtSec; }
    public double now() { return tSec; }
    public long millis() { return Math.round(tSec * 1000.0); }
}
