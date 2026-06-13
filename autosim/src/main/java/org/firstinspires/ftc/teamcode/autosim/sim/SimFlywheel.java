package org.firstinspires.ftc.teamcode.autosim.sim;

/**
 * Recording stub mirroring Flywheel's surface, with a coarse first-order spin-up so
 * {@code flywheelActualRadS} looks plausible (tunable; not a physical model).
 */
public final class SimFlywheel {
    private static final double TAU_SEC = 0.35;   // spin-up time constant

    private double target = 0;
    private double actual = 0;

    public void setTargetVelocity(double v) { target = v; }
    public void stop() { target = 0; }

    public void update(double dtSec) {
        double k = 1 - Math.exp(-dtSec / TAU_SEC);
        actual += (target - actual) * k;
    }

    public double getTarget() { return target; }
    public double getVelocityRadPerSec() { return actual; }
}
