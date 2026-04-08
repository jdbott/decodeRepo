package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    private DcMotorEx turretMotor;

    // Constants
    private final double ticksPerRev = 145.1;
    private final double gearRatio = (165 / 16.0);
    private final double ticksPerDegree = (ticksPerRev * gearRatio) / 360.0;

    // =========================
    // Old logic tunables
    // =========================
    private double kP = 0.017;
    private double kF = 0.003;
    private double minPower = 0.02;

    // =========================
    // New tracking logic tunables
    // =========================
    private double kD = 0.0008;   // velocity error gain
    private double kV = 0.003;   // desired angular velocity feedforward gain

    // Limits
    private double maxAngle = 180;
    private double minAngle = -180;

    // =========================
    // Shared state
    // =========================
    private double lastSetAngle = 0.0;
    private boolean turretBusy = false;

    // Old logic feedforward
    private double feedforward = 0.0;

    // New tracking state
    private double targetAngleDeg = 0.0;
    private double targetAngularVelDegPerSec = 0.0;

    // Which control mode to use
    private enum ControlMode {
        LEGACY_POSITION,
        TRACKING
    }

    private ControlMode controlMode = ControlMode.LEGACY_POSITION;

    // Initialization
    public void init(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turretMotor.setDirection(direction);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setLimits(double minDeg, double maxDeg) {
        minAngle = minDeg;
        maxAngle = maxDeg;
    }

    // =========================
    // Public configuration
    // =========================
    public void setKP(double newKP) { kP = newKP; }
    public void setKF(double newKF) { kF = newKF; }
    public void setKD(double newKD) { kD = newKD; }
    public void setKV(double newKV) { kV = newKV; }
    public void setMinPower(double newMinPwr) { minPower = Math.abs(newMinPwr); }

    // =========================
    // Old logic API
    // =========================
    public void setFeedforward(double angularVelDegPerSec) {
        feedforward = angularVelDegPerSec * kF;
        feedforward = Range.clip(feedforward, -1.0, 1.0);
    }

    public void setAngle(double targetDeg) {
        targetDeg = Range.clip(targetDeg, minAngle, maxAngle);
        lastSetAngle = targetDeg;
        turretBusy = true;
        controlMode = ControlMode.LEGACY_POSITION;
    }

    // =========================
    // New tracking API
    // =========================
    public void setTargetState(double targetDeg, double targetVelDegPerSec) {
        targetAngleDeg = Range.clip(targetDeg, minAngle, maxAngle);
        targetAngularVelDegPerSec = targetVelDegPerSec;
        lastSetAngle = targetAngleDeg; // keep older accessors sensible
        turretBusy = true;
        controlMode = ControlMode.TRACKING;
    }

    // =========================
    // Main update
    // =========================
    public void update() {
        if (controlMode == ControlMode.TRACKING) {
            updateTracking();
        } else {
            if (turretBusy) {
                moveTurretToAngle(lastSetAngle);
            } else {
                correctTurretPosition();
            }
        }
    }

    // =========================
    // Old logic implementation
    // =========================
    private void moveTurretToAngle(double targetAngle) {
        double targetTicks = targetAngle * ticksPerDegree;
        double currentTicks = -turretMotor.getCurrentPosition();
        double error = targetTicks - currentTicks;

        if (Math.abs(error) > (0.05 * ticksPerDegree)) { // ~0.05 deg tolerance, same as your original code
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
        double currentTicks = -turretMotor.getCurrentPosition();
        double error = targetTicks - currentTicks;

        double power = kP * error + feedforward;
        power = applyMinPower(power);
        turretMotor.setPower(Range.clip(power, -1, 1));
    }

    // =========================
    // New tracking implementation
    // =========================
    private void updateTracking() {
        double currentAngleDeg = getCurrentAngle();
        double currentVelDegPerSec = getCurrentAngularVelocityDegPerSec();

        double angleErrorDeg = targetAngleDeg - currentAngleDeg;
        double velocityErrorDegPerSec = targetAngularVelDegPerSec - currentVelDegPerSec;

        double power =
                (kP * angleErrorDeg)
                        + (kD * velocityErrorDegPerSec)
                        + (kV * targetAngularVelDegPerSec);

        power = applyMinPower(power);
        power = Range.clip(power, -1.0, 1.0);

        turretMotor.setPower(power);

        if (Math.abs(angleErrorDeg) < 0.15 && Math.abs(targetAngularVelDegPerSec) < 1.0) {
            turretBusy = false;
        } else {
            turretBusy = true;
        }
    }

    private double applyMinPower(double power) {
        if (minPower == 0) return power;
        if (power > 0 && power < minPower) return minPower;
        if (power < 0 && power > -minPower) return -minPower;
        return power;
    }

    // =========================
    // Accessors
    // =========================
    public double getCurrentAngle() {
        return -turretMotor.getCurrentPosition() / ticksPerDegree;
    }

    public double getCurrentAngularVelocityDegPerSec() {
        return -turretMotor.getVelocity() / ticksPerDegree;
    }

    public double getTargetAngle() {
        return lastSetAngle;
    }

    public double getTargetAngularVelocityDegPerSec() {
        return targetAngularVelDegPerSec;
    }

    public double getError() {
        return lastSetAngle - getCurrentAngle();
    }

    public boolean isBusy() {
        return turretBusy;
    }

    public boolean isTrackingMode() {
        return controlMode == ControlMode.TRACKING;
    }

    public void useLegacyMode() {
        controlMode = ControlMode.LEGACY_POSITION;
        targetAngularVelDegPerSec = 0.0;
    }

    public void stop() {
        turretMotor.setPower(0);
        turretBusy = false;
        targetAngularVelDegPerSec = 0.0;
        feedforward = 0.0;
    }

    public void zeroTurret() {
        turretMotor.setPower(0);
        turretBusy = false;
        feedforward = 0.0;
        targetAngularVelDegPerSec = 0.0;

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastSetAngle = 0.0;
        targetAngleDeg = 0.0;
    }
}