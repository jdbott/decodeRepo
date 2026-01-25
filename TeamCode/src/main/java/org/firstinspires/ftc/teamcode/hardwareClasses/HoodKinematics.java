package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;

public class HoodKinematics {

    private Servo hoodServo;
    private double lastGoodPos = 0.01; // safe fallback

    // Servo calibration
    private static final double SERVO_MIN = 0.01;   // corresponds to 0°
    private static final double SERVO_RANGE_DEG = 355.0; // total rotation range

    // ---------- Initialization ----------
    public void init(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(SERVO_MIN);
    }

    // ---------- Integrated ShootAngleV2 math ----------
    private static final double T = Math.tan(1.15435);

    private static double m_from_o(double o_deg) {
        double theta = Math.toRadians(o_deg);
        double t = Math.tan(theta);
        return (t * T - 1.0) / (t + T);
    }

    private static double K_of_a(double a) {
        double sqrt1024_a2 = Math.sqrt(1024 - a * a);

        double innerSqrt = Math.sqrt(
                6392558964110399635456.0 * Math.pow(a, 6)
                        - 505852289504098591440896.0 * Math.pow(a, 5)
                        + 181062374405949359673835520.0 * Math.pow(a, 4)
                        - 7273791405586025757305470976.0 * Math.pow(a, 3)
                        + 5735306921967454461179723776.0 * sqrt1024_a2 * Math.pow(a, 2)
                        + 2166278991854161643976192425984.0 * Math.pow(a, 2)
                        - 667001666737193237415494418432.0 * sqrt1024_a2 * a
                        - 69461800056188990893486340308992.0 * sqrt1024_a2
                        + 706307750271763255132160.0 * sqrt1024_a2 * Math.pow(a, 4)
                        - 88132015792670827863867392.0 * sqrt1024_a2 * Math.pow(a, 3)
                        - 114341558906155315517563120648192.0 * a
                        + 3663573612226171842562489420087296.0
        );

        double numerator = 0.5 * (
                79953480000.0 * Math.pow(a, 3)
                        + 4968105300000.0 * Math.pow(a, 2)
                        - 7813345500000.0 * sqrt1024_a2 * a
                        + 864937346850000.0 * sqrt1024_a2
                        + innerSqrt
                        - 1758218459591400.0 * a
                        + 193257407103702656.0
        );

        double denominator = 79953480000.0 * Math.pow(a, 2)
                - 9077563836000.0 * a
                + 783755172273800.0;

        return numerator / denominator;
    }

    private static double equation_for_a(double a, double o_deg) {
        double K = K_of_a(a);
        double diff = K - a;
        if (Math.abs(diff) > 224) return Double.NaN;
        double left = Math.sqrt(224 * 224 - diff * diff);
        double right = m_from_o(o_deg) * (K - 110.7) + 166.5 - Math.sqrt(32 * 32 - a * a);
        return left - right;
    }

    private static double findSingleS(double o_deg) {
        if (o_deg < -20 || o_deg > 69) return Double.NaN;

        double tol = 1e-9;
        int maxIter = 200;
        double aRoot = Double.NaN;

        double prevA = -32;
        double prevVal = equation_for_a(prevA, o_deg);

        for (double a = -31.9; a <= 32; a += 0.1) {
            double val = equation_for_a(a, o_deg);
            if (Double.isNaN(val) || Double.isNaN(prevVal)) {
                prevA = a;
                prevVal = val;
                continue;
            }

            if (prevVal * val < 0) {
                double aLow = prevA;
                double aHigh = a;
                double fLow = prevVal;
                double fHigh = val;

                for (int i = 0; i < maxIter; i++) {
                    double mid = 0.5 * (aLow + aHigh);
                    double fMid = equation_for_a(mid, o_deg);
                    if (Double.isNaN(fMid)) break;
                    if (Math.abs(fMid) < tol) {
                        aRoot = mid;
                        break;
                    }
                    if (fLow * fMid < 0) {
                        aHigh = mid;
                        fHigh = fMid;
                    } else {
                        aLow = mid;
                        fLow = fMid;
                    }
                }
                if (!Double.isNaN(aRoot)) break;
            }

            prevA = a;
            prevVal = val;
        }

        if (Double.isNaN(aRoot)) return Double.NaN;
        double s = -Math.toDegrees(Math.asin(aRoot / 32.0));
        return (s >= -36 && s <= 90) ? s : Double.NaN;
    }

    public double servoAngle(double hoodAngle) {
        return findSingleS(hoodAngle);
    }

    // ---------- Public control ----------
    public void setHoodAngle(double hoodAngleDeg) {
        try {
            double servoAngle = findSingleS(hoodAngleDeg);
            if (Double.isNaN(servoAngle)) throw new Exception("No valid hood angle");

            // Convert servo angle (0° = position 0.01) over 355° total range
            double pos = SERVO_MIN + (servoAngle / SERVO_RANGE_DEG) * (1.0 - SERVO_MIN);

            if (Double.isNaN(pos)) throw new IllegalArgumentException("Non-finite servo pos");
            if (pos < SERVO_MIN) pos = SERVO_MIN;
            if (pos > 1.0) pos = 1.0;

            hoodServo.setPosition(pos);
            lastGoodPos = pos;
        } catch (Exception e) {
            hoodServo.setPosition(lastGoodPos);
        }
    }
}