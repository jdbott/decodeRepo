package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    private DcMotorEx masterMotor, followerMotor;
    private VoltageSensor battery;

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000;
    private static final double MAX_TPS = MAX_RPM * TICKS_PER_REV / 60.0;

    // --- Control coefficients ---
    private double kP = 0.00015;          // proportional term
    private double kF_nominal = 0.0005;  // nominal feedforward tuned at 12 V

    // --- Runtime values ---
    private double targetRPM = 0.0;
    private double targetTPS = 0.0;
    private double lastPower = 0.0;
    private double lastErrorTPS = 0.0;
    private double lastFF = 0.0;
    private double lastPTerm = 0.0;
    private double lastVoltage = 12.0;

    /**
     * Initialize shooter motors
     */
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

        // get battery voltage sensor
        battery = hw.voltageSensor.iterator().next();
    }

    /**
     * Set desired velocity (RPM)
     */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        targetTPS = rpm * TICKS_PER_REV / 60.0;
    }

    /**
     * Current velocity in ticks/sec
     */
    public double getMasterVelocityTPS() {
        return masterMotor.getVelocity();
    }

    /**
     * Current velocity in RPM
     */
    public double getMasterRPM() {
        return getMasterVelocityTPS() * 60.0 / TICKS_PER_REV;
    }

    /**
     * Main velocity control loop
     */
    public void update() {
        if (masterMotor == null || followerMotor == null) return;

        double voltage = battery.getVoltage();
        lastVoltage = voltage;

        double currentTPS = getMasterVelocityTPS();
        double error = targetTPS - currentTPS;
        lastErrorTPS = error;

        // --- voltage-compensated feedforward (nominal 12 V) ---
        double kF = kF_nominal * (12.0 / voltage);
        double ff = kF * targetTPS;

        // --- proportional correction ---
        double pTerm = kP * error;
        double pCap = 0.20 * (targetTPS / MAX_TPS); // ≤20 % trim at full range
        pTerm = Range.clip(pTerm, -pCap, pCap);

        // --- combine and clamp ---
        double power = ff + pTerm;
        power = Range.clip(power, 0.0, 1.0);

        masterMotor.setPower(power);
        followerMotor.setPower(power);

        lastPower = power;
        lastFF = ff;
        lastPTerm = pTerm;
    }

    /**
     * Stop both motors
     */
    public void stop() {
        if (masterMotor != null) masterMotor.setPower(0);
        if (followerMotor != null) followerMotor.setPower(0);
        targetRPM = 0;
        targetTPS = 0;
    }

    // --- Telemetry accessors ---
    public double getLastPower() {
        return lastPower;
    }

    public double getLastErrorTPS() {
        return lastErrorTPS;
    }

    public double getLastFF() {
        return lastFF;
    }

    public double getLastPTerm() {
        return lastPTerm;
    }

    public double getLastVoltage() {
        return lastVoltage;
    }

    // --- Tuning setters ---
    public void setCoefficients(double kP, double kF_nominal) {
        this.kP = kP;
        this.kF_nominal = kF_nominal;
    }
}