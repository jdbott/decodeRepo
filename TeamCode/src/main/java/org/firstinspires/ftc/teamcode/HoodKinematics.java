package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HoodKinematics {

    private Servo hoodServo;
    private double lastGoodPos = 0.06; // safe fallback

    // Servo calibration: 0° -> 0.06, 112.5° -> 0.40
    private static final double SERVO_MIN = 0.06;
    private static final double SERVO_MAX = 0.4;
    private static final double SERVO_RANGE_DEG = 360;

    // Root-finding domain
    private static final double A_MIN = -31.99;
    private static final double A_MAX =  31.99;

    public void init(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "HoodServo");
        hoodServo.setPosition(lastGoodPos);
    }

    // ---------- Safe math helpers ----------
    private static boolean finite(double x) { return !Double.isNaN(x) && !Double.isInfinite(x); }
    private static double safesqrt(double x) { return x >= 0 ? Math.sqrt(x) : Double.NaN; }

    // ---------- Model functions (with domain checks) ----------
    static double K_of_a(double a) {
        double r1 = safesqrt(1024.0 - a*a);
        if (!finite(r1)) return Double.NaN;

        double D =
                6392558964110399635456.0*Math.pow(a,6)
                        - 505852289504098591440896.0*Math.pow(a,5)
                        + 181062374405949359673835520.0*Math.pow(a,4)
                        - 7273791405586025757305470976.0*Math.pow(a,3)
                        + 5735306921967454461179723776.0*r1*Math.pow(a,2)
                        + 2166278991854161643976192425984.0*Math.pow(a,2)
                        - 667001666737193237415494418432.0*r1*a
                        - 69461800056188990893486340308992.0*r1
                        + 706307750271763255132160.0*r1*Math.pow(a,4)
                        - 88132015792670827863867392.0*r1*Math.pow(a,3)
                        - 114341558906155315517563120648192.0*a
                        + 3663573612226171842562489420087296.0;

        double inner = safesqrt(D);
        if (!finite(inner)) return Double.NaN;

        double numerator = 0.5*(
                79953480000.0*Math.pow(a,3)
                        + 4968105300000.0*Math.pow(a,2)
                        - 7813345500000.0*r1*a
                        + 864937346850000.0*r1
                        + inner
                        - 1758218459591400.0*a
                        + 193257407103702656.0
        );

        double denominator =
                79953480000.0*Math.pow(a,2)
                        - 9077563836000.0*a
                        + 783755172273800.0;

        return numerator / denominator;
    }

    static final double T = Math.tan(1.15435);

    static double m_from_o(double o_deg) {
        double t = Math.tan(Math.toRadians(o_deg));
        return (t*T - 1.0) / (t + T);
    }

    static double equation_for_a(double a, double o_deg) {
        double K = K_of_a(a);
        if (!finite(K)) return Double.NaN;

        double diff = K - a;
        double left = safesqrt(224.0*224.0 - diff*diff);
        double right = m_from_o(o_deg)*(K - 110.7) + 166.5 - safesqrt(32.0*32.0 - a*a);

        double val = left - right;
        return finite(val) ? val : Double.NaN;
    }

    // ---------- Robust bracket + bisection ----------
    static double findSingleS(double o_deg) throws Exception {
        // 1) Scan for a finite sign-change bracket
        double step = 0.25;
        Double aPrev = null, fPrev = null;

        for (double a = A_MIN; a <= A_MAX; a += step) {
            double f = equation_for_a(a, o_deg);
            if (!finite(f)) continue; // skip invalid points

            if (aPrev != null && finite(fPrev)) {
                if (f == 0.0) { aPrev = a - 1e-6; fPrev = equation_for_a(aPrev, o_deg); }
                if (fPrev != null && finite(fPrev) && fPrev * f <= 0.0) {
                    // 2) Bisection inside [aPrev, a]
                    double lo = aPrev, hi = a;
                    double fLo = fPrev, fHi = f;
                    for (int i = 0; i < 100; i++) {
                        double mid = 0.5*(lo + hi);
                        double fMid = equation_for_a(mid, o_deg);
                        if (!finite(fMid)) { // nudge off singularities
                            mid = Math.nextAfter(mid, hi);
                            fMid = equation_for_a(mid, o_deg);
                            if (!finite(fMid)) { mid = Math.nextAfter(mid, lo); fMid = equation_for_a(mid, o_deg); }
                            if (!finite(fMid)) { break; }
                        }
                        if (Math.abs(fMid) < 1e-9) { lo = hi = mid; break; }
                        if (fLo * fMid <= 0.0) { hi = mid; fHi = fMid; } else { lo = mid; fLo = fMid; }
                    }
                    double aRoot = 0.5*(lo + hi);
                    // 3) Convert a -> servo angle s
                    double s = -Math.toDegrees(Math.asin(aRoot / 32.0));
                    if (!finite(s)) throw new Exception("asin domain error");
                    return s;
                }
            }
            aPrev = a; fPrev = f;
        }
        throw new Exception("No valid bracket for hood angle " + o_deg);
    }

    // ---------- Public control ----------
    public void setHoodAngle(double hoodAngleDeg) {
        try {
            double servoAngle = findSingleS(hoodAngleDeg);          // [-90, +90] model angle
            double physicalDeg = (90.0 - servoAngle) * (SERVO_RANGE_DEG / 180.0);
            double pos = SERVO_MIN + (physicalDeg / SERVO_RANGE_DEG) * (SERVO_MAX - SERVO_MIN);

            if (!finite(pos)) throw new IllegalArgumentException("Non-finite servo pos");
            // Clamp hard
            if (pos < SERVO_MIN) pos = SERVO_MIN;
            if (pos > SERVO_MAX) pos = SERVO_MAX;

            hoodServo.setPosition(pos);
            lastGoodPos = pos;
        } catch (Exception e) {
            // Hold last good rather than jump to hard-coded center
            hoodServo.setPosition(lastGoodPos);
        }
    }
}