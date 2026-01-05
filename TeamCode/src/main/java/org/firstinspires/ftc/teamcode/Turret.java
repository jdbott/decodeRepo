package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    private DcMotorEx turretMotor;

    // Constants
    private final double ticksPerRev = 145.1;
    private final double gearRatio = (108.0 / 18.0) * 2.0; // 12:1 motor revs per turret rev
    private final double ticksPerDegree = (ticksPerRev * gearRatio) / 360.0;

    // Tunable parameters
    private double kP = 0.01;
    private double kF = 0.003; // feedforward gain (adjust in test)
    private double minPower = 0.05;
    private double maxAngle = 200;
    private double minAngle = -160;

    // State variables
    private double lastSetAngle = 0;  // in degrees
    private boolean turretBusy = false;
    private double feedforward = 0;   // added for angular velocity compensation

    // Initialization
    public void init(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turretMotor.setDirection(direction);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setLimits(minAngle, maxAngle);
    }

    // ===== Public Configuration =====
    public void setKP(double newKP) { kP = newKP; }
    public void setKF(double newKF) { kF = newKF; }
    public void setMinPower(double minPwr) { minPower = Math.abs(minPwr); }
    public void setLimits(double minDeg, double maxDeg) { minAngle = minDeg; maxAngle = maxDeg; }

    // Feedforward setter
    public void setFeedforward(double angularVelDegPerSec) {
        // Oppose robot rotation to maintain heading
        feedforward = angularVelDegPerSec * kF;
        feedforward = Range.clip(feedforward, -1.0, 1.0);
    }

    // ===== Motion Control =====
    public void setAngle(double targetDeg) {
        targetDeg = Range.clip(targetDeg, minAngle, maxAngle);
        lastSetAngle = targetDeg;
        turretBusy = true;
    }

    public void update() {
        if (turretBusy) moveTurretToAngle(lastSetAngle);
        else correctTurretPosition();
    }

    private void moveTurretToAngle(double targetAngle) {
        double targetTicks = targetAngle * ticksPerDegree;
        double currentTicks = turretMotor.getCurrentPosition();
        double error = targetTicks - currentTicks;

        if (Math.abs(error) > (0.25 * ticksPerDegree)) { // >1° tolerance
            double power = kP * error + feedforward;
            power = applyMinPower(power);
            turretMotor.setPower(Range.clip(power, -1, 1));
        } else {
            turretMotor.setPower(0);
            turretBusy = false;
        }
    }

    private void correctTurretPosition() {
        double targetTicks = lastSetAngle * ticksPerDegree;
        double currentTicks = turretMotor.getCurrentPosition();
        double error = targetTicks - currentTicks;
        double power = kP * error + feedforward;
        power = applyMinPower(power);
        turretMotor.setPower(Range.clip(power, -1, 1));
    }

    private double applyMinPower(double power) {
        if (minPower == 0) return power;
        if (power > 0 && power < minPower) return minPower;
        if (power < 0 && power > -minPower) return -minPower;
        return power;
    }

    // ===== Accessors =====
    public double getCurrentAngle() { return turretMotor.getCurrentPosition() / ticksPerDegree; }
    public double getTargetAngle() { return lastSetAngle; }
    public double getError() { return lastSetAngle - getCurrentAngle(); }
    public boolean isBusy() { return turretBusy; }

    public void stop() {
        turretMotor.setPower(0);
        turretBusy = false;
    }

    public void zeroTurret() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lastSetAngle = 0;
    }
}