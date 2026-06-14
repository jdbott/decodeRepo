package org.firstinspires.ftc.teamcode.autosim.sim;

import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;
import org.firstinspires.ftc.teamcode.autosim.geom.Pt;
import org.firstinspires.ftc.teamcode.autosim.model.PathSpec;
import org.firstinspires.ftc.teamcode.autosim.model.SimTrace;

import java.util.List;

/**
 * Coarse point-mass follower (AUTOSIM_DESIGN.md §5). On {@code followPath} it computes the
 * path arc length and a trapezoidal accel/cruise/decel traversal time, then interpolates the
 * pose along the sampled polyline as the virtual clock advances. Heading is the path's
 * constant-interpolation target. {@code isBusy()} is true until arrival.
 *
 * <p>This validates FSM logic, ordering, and timing — NOT exact pose error. CRUISE/ACCEL are
 * deliberately coarse, seeded near Pedro's measured caps; swap this one class for higher
 * fidelity later without touching the FSM.
 */
public final class SimFollower {
    private static final double CRUISE_IN_S = 55.0;   // ~ below Pedro xVel 73.8 / yVel 60.8
    private static final double ACCEL_IN_S2 = 80.0;   // accel/decel magnitude

    private final SimClock clock;
    private final SimTrace trace;

    private PathSpec active;
    private double startSec;
    private double travSec;
    private double length;
    private Pose2d lastPose = new Pose2d(0, 0, 0);

    public SimFollower(SimClock clock, SimTrace trace) {
        this.clock = clock;
        this.trace = trace;
    }

    public void setStartingPose(Pose2d p) { lastPose = p; active = null; }
    public void setMaxPower(double p) {}
    public void update() {}

    /** Begin following a (pre-sampled) path; stamps its timing and records it in the trace. */
    public void followPath(PathSpec spec, boolean holdEnd) {
        active = spec;
        startSec = clock.now();
        length = arcLength(spec.polyline);
        travSec = traversalTime(length);
        spec.tStartMs = clock.millis();
        spec.tEndMs = clock.millis() + Math.round(travSec * 1000.0);
        trace.paths.add(spec);
    }

    public boolean isBusy() {
        return active != null && (clock.now() - startSec) < travSec;
    }

    public Pose2d getPose() {
        if (active == null) return lastPose;
        double elapsed = clock.now() - startSec;
        double d = (elapsed >= travSec) ? length : distanceAt(elapsed);
        Pt p = pointAtArcLength(active.polyline, d);
        Pose2d pose = new Pose2d(p.x, p.y, active.headingDeg);
        if (elapsed >= travSec) lastPose = pose;   // hold at the end between paths
        return pose;
    }

    // ----- trapezoidal profile -----

    private double traversalTime(double D) {
        double v = CRUISE_IN_S, a = ACCEL_IN_S2;
        double tAcc = v / a, dAcc = 0.5 * a * tAcc * tAcc;
        if (2 * dAcc >= D) return 2 * Math.sqrt(D / a);          // triangular
        return 2 * tAcc + (D - 2 * dAcc) / v;                    // trapezoid
    }

    private double distanceAt(double t) {
        double v = CRUISE_IN_S, a = ACCEL_IN_S2, D = length;
        double tAcc = v / a, dAcc = 0.5 * a * tAcc * tAcc;
        if (2 * dAcc >= D) {                                     // triangular
            double tPeak = Math.sqrt(D / a), T = 2 * tPeak;
            if (t <= 0) return 0;
            if (t >= T) return D;
            return (t < tPeak) ? 0.5 * a * t * t : D - 0.5 * a * (T - t) * (T - t);
        }
        double tCruise = (D - 2 * dAcc) / v, T = 2 * tAcc + tCruise;
        if (t <= 0) return 0;
        if (t >= T) return D;
        if (t < tAcc) return 0.5 * a * t * t;
        if (t < tAcc + tCruise) return dAcc + v * (t - tAcc);
        double td = T - t;
        return D - 0.5 * a * td * td;
    }

    // ----- polyline helpers -----

    static double arcLength(List<Pt> poly) {
        double s = 0;
        for (int i = 1; i < poly.size(); i++) s += dist(poly.get(i - 1), poly.get(i));
        return s;
    }

    static Pt pointAtArcLength(List<Pt> poly, double d) {
        if (d <= 0) return poly.get(0);
        double acc = 0;
        for (int i = 1; i < poly.size(); i++) {
            double seg = dist(poly.get(i - 1), poly.get(i));
            if (acc + seg >= d) {
                double f = seg == 0 ? 0 : (d - acc) / seg;
                Pt a = poly.get(i - 1), b = poly.get(i);
                return new Pt(a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f);
            }
            acc += seg;
        }
        return poly.get(poly.size() - 1);
    }

    static double dist(Pt a, Pt b) {
        double dx = b.x - a.x, dy = b.y - a.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
