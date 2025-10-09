package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    private DcMotorEx masterMotor, followerMotor;
    private static final double TICKS_PER_REV = 28.0;

    // --- Tuned coefficients for 4900 RPM max ---
    private double kP = 0.00012;    // proportional trim
    private double kF = 0.000437;   // 1 / (4900 RPM × 28 / 60)
    private double kS0 = 0.08;      // base static term at low RPM

    // --- Constants for scaling ---
    private static final double MAX_RPM = 4900.0;
    private static final double MAX_TPS = MAX_RPM * TICKS_PER_REV / 60.0;

    // --- State tracking ---
    private double targetRPM = 0.0;
    private double targetTPS = 0.0;
    private double lastPower = 0.0;
    private double lastErrorTPS = 0.0;
    private double lastFF = 0.0;
    private double lastPTerm = 0.0;
    private double lastKSEff = 0.0;

    /** Initialize shooter motors for open-loop velocity control */
    public void init(HardwareMap hw,
                     String masterName, String followerName,
                     DcMotorSimple.Direction masterDir, DcMotorSimple.Direction followerDir) {

        masterMotor = hw.get(DcMotorEx.class, masterName);
        followerMotor = hw.get(DcMotorEx.class, followerName);

        masterMotor.setDirection(masterDir);
        followerMotor.setDirection(followerDir);

        masterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        masterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        followerMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        masterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        followerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /** Set desired velocity (RPM) */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        targetTPS = rpm * TICKS_PER_REV / 60.0;
    }

    /** Get current velocity in ticks/sec */
    public double getMasterVelocityTPS() {
        return masterMotor.getVelocity();
    }

    /** Get current velocity in RPM */
    public double getMasterRPM() {
        return getMasterVelocityTPS() * 60.0 / TICKS_PER_REV;
    }

    /** Velocity control loop — call each cycle */
    public void update() {
        if (masterMotor == null || followerMotor == null) return;

        double currentTPS = getMasterVelocityTPS();
        double error = targetTPS - currentTPS;
        lastErrorTPS = error;

        // --- Dynamic kS: fades out linearly to 0 by 3000 RPM ---
        double kS_eff = 0.0;
        if (targetRPM > 0) {
            double fade = Math.max(0.0, 1.0 - (targetRPM / 3000.0));
            kS_eff = kS0 * fade;
        }
        lastKSEff = kS_eff;

        // --- Feedforward and proportional terms ---
        double ff = kF * targetTPS;
        double pTerm = kP * error;

        // --- Cap proportional term to avoid mid-range overshoot ---
        double pCap = 0.20 * (targetTPS / MAX_TPS); // ≤20% trim at full range
        pTerm = Range.clip(pTerm, -pCap, pCap);

        // --- Combine ---
        double power = (targetTPS > 0 ? kS_eff : 0.0) + ff + pTerm;
        power = Range.clip(power, 0.0, 1.0);

        masterMotor.setPower(power);
        followerMotor.setPower(power);

        // --- Record for telemetry ---
        lastPower = power;
        lastFF = ff;
        lastPTerm = pTerm;
    }

    /** Stop both motors */
    public void stop() {
        if (masterMotor != null) masterMotor.setPower(0);
        if (followerMotor != null) followerMotor.setPower(0);
        targetRPM = 0;
        targetTPS = 0;
    }

    // --- Telemetry accessors ---
    public double getLastPower() { return lastPower; }
    public double getLastErrorTPS() { return lastErrorTPS; }
    public double getLastFF() { return lastFF; }
    public double getLastPTerm() { return lastPTerm; }
    public double getLastKSEff() { return lastKSEff; }
    public double getkP() { return kP; }
    public double getkF() { return kF; }
    public double getkS0() { return kS0; }

    // --- Manual tuning setters ---
    public void setCoefficients(double kP, double kF) {
        this.kP = kP;
        this.kF = kF;
    }

    public void setStatic(double kS0) {
        this.kS0 = kS0;
    }
}