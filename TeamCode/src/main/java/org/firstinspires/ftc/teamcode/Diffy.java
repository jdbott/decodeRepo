package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Diffy {
    private final Servo s1, s2;

    // Pitch: 1 servo-degree per output-degree (±90° mech range about mid)
    private double ALPHA = 1.0;

    // Roll: servo-deg per output-deg. 52T (driver) : 18T (driven) -> 18/52
    private double BETA  = 18.0 / 52.0; // ≈0.3461538

    // Flip if roll direction is backwards
    private int rollSign = +1;

    // Midpoint servo positions measured at P=90°, R=0°
    private double mid1 = 0.50, mid2 = 0.50;

    private static final double SERVO_TOTAL_DEG = 270.0;
    private static final double SERVO_HALF = SERVO_TOTAL_DEG / 2.0; // 135°

    // Mechanism limits
    private static final double PITCH_ABS_MAX = 90.0;   // ±90°
    private static final double ROLL_ABS_MAX  = 180.0;  // ±180°

    public Diffy(HardwareMap hw, String s1Name, String s2Name) {
        s1 = hw.get(Servo.class, s1Name);
        s2 = hw.get(Servo.class, s2Name);
        s1.setDirection(Servo.Direction.REVERSE);
    }

    /** Call while mechanism is at P=90°, R=0°. */
    public void setMidpoints(double servo1Mid, double servo2Mid) {
        this.mid1 = servo1Mid;
        this.mid2 = servo2Mid;
    }

    /** Optional: tweak mixes after confirming geometry. */
    public void setMix(double alphaServoDegPerPitchDeg, double betaServoDegPerRollDeg) {
        this.ALPHA = alphaServoDegPerPitchDeg;
        this.BETA  = betaServoDegPerRollDeg;
    }

    /** Optional: invert roll if needed. */
    public void setRollDirection(boolean invert) { this.rollSign = invert ? -1 : +1; }

    /** Command pitch Pdeg in [0..180], roll Rdeg in [-180..+180]. */
    public void setPitchRoll(double Pdeg, double Rdeg) {
        double p = clamp(Pdeg, 0, 180) - 90.0;                     // [-90..+90]
        p = clamp(p, -PITCH_ABS_MAX, PITCH_ABS_MAX);

        double r = clamp(Rdeg, -ROLL_ABS_MAX, ROLL_ABS_MAX) * rollSign;

        // Mix in SERVO DEGREES about each mid
        double s1deg =  ALPHA * p + BETA * r;
        double s2deg =  ALPHA * p - BETA * r;

        // Enforce servo travel
        double maxAbs = Math.max(Math.abs(s1deg), Math.abs(s2deg));
        if (maxAbs > SERVO_HALF) {
            double scale = SERVO_HALF / maxAbs;
            s1deg *= scale;
            s2deg *= scale;
        }

        // Map to [0..1] and clamp
        double pos1 = clamp01(mid1 + (s1deg / SERVO_TOTAL_DEG));
        double pos2 = clamp01(mid2 + (s2deg / SERVO_TOTAL_DEG));

        s1.setPosition(pos1);
        s2.setPosition(pos2);
    }

    public void setRollOnly(double currentPdeg, double Rdeg) { setPitchRoll(currentPdeg, Rdeg); }
    public void setPitchOnly(double Pdeg, double currentRdeg) { setPitchRoll(Pdeg, currentRdeg); }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double clamp01(double v) { return clamp(v, 0.0, 1.0); }
}