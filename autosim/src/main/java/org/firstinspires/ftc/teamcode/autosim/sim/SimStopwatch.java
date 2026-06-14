package org.firstinspires.ftc.teamcode.autosim.sim;

/** Drop-in for FTC's {@code ElapsedTime}, but reads the virtual {@link SimClock}. */
public final class SimStopwatch {
    private final SimClock clock;
    private double t0;

    public SimStopwatch(SimClock clock) {
        this.clock = clock;
        this.t0 = clock.now();
    }

    public void reset() { t0 = clock.now(); }
    public double seconds() { return clock.now() - t0; }
}
