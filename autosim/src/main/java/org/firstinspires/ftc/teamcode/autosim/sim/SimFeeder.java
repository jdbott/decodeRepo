package org.firstinspires.ftc.teamcode.autosim.sim;

/**
 * Recording stub mirroring Feeder's surface. Tracks the latest arm/clutch commands so
 * Phase 3 can emit feed/shoot events; Phase 2 doesn't read these.
 */
public final class SimFeeder {
    public String arm = "block";    // "block" | "shoot"
    public String clutch = "in";    // "in" | "out"

    public void armShoot() { arm = "shoot"; }
    public void armBlock() { arm = "block"; }
    public void clutchIn() { clutch = "in"; }
    public void clutchOut() { clutch = "out"; }
}
