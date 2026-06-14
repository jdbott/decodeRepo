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
 * tangent / linear), reproducing Pedro's interpolation. At the start of a path, the robot
 * performs a blocking in-place pivot to the path's initial commanded heading (at
 * {@link #TURN_RATE_DEG_S}) before translation begins — "turn, then move," as before. A
 * mid-path {@link PathSpec#thenConstant} heading-mode switch, however, does not block: the
 * robot's actual heading continuously chases the new commanded heading at up to
 * {@link #TURN_RATE_DEG_S} while translation continues — "turn and move at the same time."
 *
 * <p>This validates FSM logic, ordering, and timing — NOT exact pose error. CRUISE/ACCEL are
 * deliberately coarse, seeded near Pedro's measured caps; swap this one class for higher
 * fidelity later without touching the FSM.
 */
public final class SimFollower {
    private static final double CRUISE_IN_S = 55.0;     // ~ below Pedro xVel 73.8 / yVel 60.8
    private static final double ACCEL_IN_S2 = 80.0;     // accel/decel magnitude
    private static final double TURN_RATE_DEG_S = 270.0; // max heading slew rate while driving

    private final SimClock clock;
    private final SimTrace trace;

    private PathSpec active;
    private double startSec;
    private double travSec;
    private double length;
    private double activeCruise = CRUISE_IN_S;
    private double maxPower = 1.0;
    private Pose2d lastPose = new Pose2d(0, 0, 0);
    private double lastSampleSec = 0;   // clock time of the last getPose() call, for switch heading-rate limiting

    // Blocking start-of-path pivot.
    private double turnSec;
    private double turnFromDeg;
    private double turnToDeg;

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

    /** Fraction of the active path traveled, 0..1 (a proxy for Pedro's parametric t-value). */
    public double getCurrentTValue() {
        if (active == null || travSec <= 0) return 1.0;
        double f = (clock.now() - startSec - turnSec) / travSec;
        if (f < 0) return 0;
        return f > 1 ? 1 : f;
    }

    public Pose2d getPose() {
        if (active == null) { lastSampleSec = clock.now(); return lastPose; }
        double elapsed = clock.now() - startSec;

        if (elapsed < turnSec) {
            // Blocking in-place pivot to the path's initial commanded heading.
            Pt p0 = pointAtArcLength(active.polyline, 0);
            double dtSec = Math.max(0, clock.now() - lastSampleSec);
            double maxStep = TURN_RATE_DEG_S * dtSec;
            double delta = shortestDelta(lastPose.headingDeg, turnToDeg);
            double step = Math.max(-maxStep, Math.min(maxStep, delta));
            Pose2d pose = new Pose2d(p0.x, p0.y, lastPose.headingDeg + step);
            lastPose = pose;
            lastSampleSec = clock.now();
            return pose;
        }

        double travElapsed = elapsed - turnSec;
        double d = (travElapsed >= travSec) ? length : distanceAt(travElapsed);
        Pt p = pointAtArcLength(active.polyline, d);
        double target = headingAt(active, d);

        double t = length <= 0 ? 1.0 : d / length;
        double heading;
        if (active.headingSwitchT >= 0 && t > active.headingSwitchT) {
            // Mid-path thenConstant switch: chase the new heading concurrently with translation.
            double dtSec = Math.max(0, clock.now() - lastSampleSec);
            double maxStep = TURN_RATE_DEG_S * dtSec;
            double delta = shortestDelta(lastPose.headingDeg, target);
            double step = Math.max(-maxStep, Math.min(maxStep, delta));
            heading = lastPose.headingDeg + step;
        } else {
            heading = target;
        }

        Pose2d pose = new Pose2d(p.x, p.y, heading);
        lastPose = pose;
        lastSampleSec = clock.now();
        return pose;
    }

    // ----- heading interpolation -----

    /** Heading (deg) the path commands at arc-length {@code d} along its polyline. */
    private double headingAt(PathSpec s, double d) {
        double t = length <= 0 ? 1.0 : d / length;
        if (s.headingSwitchT >= 0 && t > s.headingSwitchT) {
            return headingForMode(s.headingModeAfter, s, d, t, s.headingDegAfter);
        }
        return headingForMode(s.headingMode, s, d, t, s.headingDeg);
    }

    /** Heading (deg) {@code mode} commands at arc-length {@code d} (path fraction {@code t}). */
    private double headingForMode(HeadingMode mode, PathSpec s, double d, double t, double constDeg) {
        switch (mode) {
            case TANGENT:         return tangentAt(s.polyline, d);
            case REVERSE_TANGENT: return tangentAt(s.polyline, d) + 180.0;
            case LINEAR: {
                double f = s.headingEndT <= 0 ? 1.0 : Math.min(1.0, t / s.headingEndT);
                return s.headingStartDeg + shortestDelta(s.headingStartDeg, s.headingEndDeg) * f;
            }
            case CONSTANT:
            default:              return constDeg;
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
