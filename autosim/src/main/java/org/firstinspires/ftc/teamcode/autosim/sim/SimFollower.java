package org.firstinspires.ftc.teamcode.autosim.sim;

import org.firstinspires.ftc.teamcode.autosim.geom.Pose2d;
import org.firstinspires.ftc.teamcode.autosim.geom.Pt;
import org.firstinspires.ftc.teamcode.autosim.model.HeadingMode;
import org.firstinspires.ftc.teamcode.autosim.model.PathSpec;
import org.firstinspires.ftc.teamcode.autosim.model.SimTrace;

import java.util.List;

/**
 * Coarse point-mass follower (AUTOSIM_DESIGN.md §5). On {@code followPath} it computes the
 * path arc length and a trapezoidal accel/cruise/decel traversal time, then interpolates the
 * pose along the sampled polyline as the virtual clock advances. {@code isBusy()} is true until
 * arrival.
 *
 * <p><b>Heading.</b> Each path declares a {@link HeadingMode} (constant / tangent / reversed
 * tangent / linear), reproducing Pedro's interpolation. Rather than snapping, the robot first
 * pivots in place from its current heading to the path's starting heading — a turn phase that
 * consumes sim time ({@link #TURN_RATE_DEG_S}) — and only then translates, so playback shows the
 * real "turn, then drive" behavior. Heading then tracks the mode point-by-point along the path.
 *
 * <p>This validates FSM logic, ordering, and timing — NOT exact pose error. CRUISE/ACCEL are
 * deliberately coarse, seeded near Pedro's measured caps; swap this one class for higher
 * fidelity later without touching the FSM.
 */
public final class SimFollower {
    private static final double CRUISE_IN_S = 55.0;     // ~ below Pedro xVel 73.8 / yVel 60.8
    private static final double ACCEL_IN_S2 = 80.0;     // accel/decel magnitude
    private static final double TURN_RATE_DEG_S = 270.0; // in-place pivot speed before translating

    private final SimClock clock;
    private final SimTrace trace;

    private PathSpec active;
    private double startSec;
    private double travSec;
    private double turnSec;        // in-place pivot time before translation begins
    private double turnFromDeg;    // heading at the moment the path started
    private double turnToDeg;      // path's heading at its start point
    private double length;
    private double activeCruise = CRUISE_IN_S;
    private double maxPower = 1.0;
    private Pose2d lastPose = new Pose2d(0, 0, 0);

    public SimFollower(SimClock clock, SimTrace trace) {
        this.clock = clock;
        this.trace = trace;
    }

    public void setStartingPose(Pose2d p) { lastPose = p; active = null; }
    /** Scales the cruise speed of subsequently-started paths (0..1), like Pedro's max power. */
    public void setMaxPower(double p) { maxPower = Math.max(0.05, Math.min(1.0, p)); }
    public void update() {}
    public boolean isRobotStuck() { return false; }   // the kinematic model never stalls
    public void breakFollowing() { lastPose = getPose(); active = null; }

    /** Begin following a (pre-sampled) path; stamps its timing and records it in the trace. */
    public void followPath(PathSpec spec, boolean holdEnd) {
        active = spec;
        startSec = clock.now();
        length = arcLength(spec.polyline);
        activeCruise = CRUISE_IN_S * maxPower;
        travSec = traversalTime(length);

        // Turn-then-move: pivot in place from the current heading to this path's starting heading.
        turnFromDeg = lastPose.headingDeg;
        turnToDeg = headingAt(spec, 0.0);
        turnSec = Math.abs(shortestDelta(turnFromDeg, turnToDeg)) / TURN_RATE_DEG_S;

        spec.tStartMs = clock.millis();
        spec.tEndMs = clock.millis() + Math.round((turnSec + travSec) * 1000.0);
        trace.paths.add(spec);
    }

    public boolean isBusy() {
        return active != null && (clock.now() - startSec) < turnSec + travSec;
    }

    /**
     * Fraction of the active path <i>translated</i>, 0..1 (a proxy for Pedro's parametric t-value).
     * Stays 0 during the initial in-place pivot, so FSM t-value gates fire on real path progress.
     */
    public double getCurrentTValue() {
        if (active == null || travSec <= 0) return 1.0;
        double moveElapsed = (clock.now() - startSec) - turnSec;
        if (moveElapsed <= 0) return 0;
        double f = moveElapsed / travSec;
        return f > 1 ? 1 : f;
    }

    public Pose2d getPose() {
        if (active == null) return lastPose;
        double elapsed = clock.now() - startSec;

        Pose2d pose;
        if (elapsed < turnSec) {                          // pivoting in place at the path start
            double f = turnSec <= 0 ? 1.0 : elapsed / turnSec;
            double h = turnFromDeg + shortestDelta(turnFromDeg, turnToDeg) * f;
            Pt p0 = active.polyline.get(0);
            pose = new Pose2d(p0.x, p0.y, h);
        } else {
            double moveElapsed = elapsed - turnSec;
            double d = (moveElapsed >= travSec) ? length : distanceAt(moveElapsed);
            Pt p = pointAtArcLength(active.polyline, d);
            pose = new Pose2d(p.x, p.y, headingAt(active, d));
        }
        // Track the latest pose so the next path's pivot starts from the true current heading,
        // even when the FSM chains paths in a single tick (no idle frame to refresh it).
        lastPose = pose;
        return pose;
    }

    // ----- heading interpolation -----

    /** Heading (deg) the path commands at arc-length {@code d} along its polyline. */
    private double headingAt(PathSpec s, double d) {
        switch (s.headingMode) {
            case TANGENT:         return tangentAt(s.polyline, d);
            case REVERSE_TANGENT: return tangentAt(s.polyline, d) + 180.0;
            case LINEAR: {
                double t = length <= 0 ? 1.0 : d / length;
                double f = s.headingEndT <= 0 ? 1.0 : Math.min(1.0, t / s.headingEndT);
                return s.headingStartDeg + shortestDelta(s.headingStartDeg, s.headingEndDeg) * f;
            }
            case CONSTANT:
            default:              return s.headingDeg;
        }
    }

    /** Direction (deg) of the polyline segment containing arc-length {@code d}. */
    static double tangentAt(List<Pt> poly, double d) {
        if (poly.size() < 2) return 0;
        double acc = 0;
        for (int i = 1; i < poly.size(); i++) {
            double seg = dist(poly.get(i - 1), poly.get(i));
            if (acc + seg >= d && seg > 0) {
                Pt a = poly.get(i - 1), b = poly.get(i);
                return Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
            }
            acc += seg;
        }
        Pt a = poly.get(poly.size() - 2), b = poly.get(poly.size() - 1);
        return Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
    }

    /** Smallest signed angle (deg) carrying {@code from} onto {@code to}, in (-180, 180]. */
    static double shortestDelta(double from, double to) {
        double d = (to - from) % 360.0;
        if (d > 180) d -= 360;
        if (d <= -180) d += 360;
        return d;
    }

    // ----- trapezoidal profile -----

    private double traversalTime(double D) {
        double v = activeCruise, a = ACCEL_IN_S2;
        double tAcc = v / a, dAcc = 0.5 * a * tAcc * tAcc;
        if (2 * dAcc >= D) return 2 * Math.sqrt(D / a);          // triangular
        return 2 * tAcc + (D - 2 * dAcc) / v;                    // trapezoid
    }

    private double distanceAt(double t) {
        double v = activeCruise, a = ACCEL_IN_S2, D = length;
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
