package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    private DcMotorEx turretMotor;

    // Constants
    private final double ticksPerRev = 384.5;
    private final double gearRatio = 108.0 / 24.0; // Driven / Driver = 4.5:1
    private final double ticksPerDegree = (ticksPerRev * gearRatio) / 360.0;

    // Tunable parameters
    private double kP = 0.02;
    private double minPower = 0;
    private double maxAngle = 180;
    private double minAngle = -180;

    // State variables
    private double lastSetAngle = 0;  // in degrees
    private boolean turretBusy = false;

    // Initialization
    public void init(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turretMotor.setDirection(direction);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // ===== Public Configuration =====

    public void setKP(double newKP) {
        kP = newKP;
    }

    public void setMinPower(double minPwr) {
        minPower = Math.abs(minPwr);
    }

    public void setLimits(double minDeg, double maxDeg) {
        minAngle = minDeg;
        maxAngle = maxDeg;
    }

    // ===== Motion Control =====

    public void setAngle(double targetDeg) {
        // Clamp within mechanical limits
        targetDeg = Range.clip(targetDeg, minAngle, maxAngle);
        lastSetAngle = targetDeg;
        turretBusy = true;
    }

    // Called periodically from loop()
    public void update() {
        if (turretBusy) {
            moveTurretToAngle(lastSetAngle);
        } else {
            correctTurretPosition();
        }
    }

    // Move to a target position in degrees
    private void moveTurretToAngle(double targetAngle) {
        double targetTicks = targetAngle * ticksPerDegree;
        double currentTicks = turretMotor.getCurrentPosition();
        double error = targetTicks - currentTicks;

        if (Math.abs(error) > ticksPerDegree) { // within 1 degree tolerance
            double power = kP * error;
            power = applyMinPower(power);
            turretMotor.setPower(Range.clip(power, -1, 1));
        } else {
            turretMotor.setPower(0);
            turretBusy = false;
        }
    }

    // Hold position (correct small drift)
    private void correctTurretPosition() {
        double targetTicks = lastSetAngle * ticksPerDegree;
        double currentTicks = turretMotor.getCurrentPosition();
        double error = targetTicks - currentTicks;
        double power = 0.02 * error;
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

    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / ticksPerDegree;
    }

    public double getTargetAngle() {
        return lastSetAngle;
    }

    public double getError() {
        return lastSetAngle - getCurrentAngle();
    }

    public boolean isBusy() {
        return turretBusy;
    }

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