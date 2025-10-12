package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class ServoController {
    // Hardware
    private final List<CRServo> servos = new ArrayList<>();
    private final AnalogInput encoder;

    // Encoder tracking
    private double zeroOffsetDeg = 0.0;
    private double lastZeroedDeg = 0.0;
    private int revolutions = 0;

    // Per-servo configuration
    private final List<Double> dirSign = new ArrayList<>();
    private final List<Double> lastSetTargetDeg = new ArrayList<>();

    // PID terms
    private double kP = 0.01;
    private double kD = 0.002;
    private double lastError = 0.0;
    private double minPower = 0.0;

    private boolean servosBusy = false;

    // Constants
    private static final double TOLERANCE_DEG = 4;
    private static final double VREF = 3.3;
    private static final double WRAP_THRESH_DEG = 270.0;

    // Gear ratio (servo gear : output gear = 30 : 36)
    private static final double GEAR_RATIO = 30.0 / 36.0;
    private static final double INV_GEAR_RATIO = 1.0 / GEAR_RATIO;

    public ServoController(HardwareMap hw, String[] servoNames, double[] directions, String encoderName) {
        if (servoNames.length != directions.length)
            throw new IllegalArgumentException("Servo names and directions arrays must match length.");

        encoder = hw.get(AnalogInput.class, encoderName);
        for (int i = 0; i < servoNames.length; i++) {
            servos.add(hw.get(CRServo.class, servoNames[i]));
            dirSign.add(Math.signum(directions[i]));
            lastSetTargetDeg.add(0.0);
        }
        zeroNow();
    }

    // --- Configuration ---
    public void setKP(double gain) { kP = gain; }
    public void setKD(double gain) { kD = gain; }
    public void setMinPower(double pwr) { minPower = Math.abs(pwr); }

    // --- Zero encoder ---
    public void zeroNow() {
        zeroOffsetDeg = rawAngleDeg();
        lastZeroedDeg = 0.0;
        revolutions = 0;
        for (CRServo s : servos) s.setPower(0);
        servosBusy = false;
        lastError = 0.0;
    }

    // --- Commands ---
    public void moveServosToPosition(double targetOutputDeg) {
        double targetWrapped = wrapTo0_360(targetOutputDeg);
        double currentOutput = getContinuousOutputAngleDeg();
        double nearest = nearestEquivalent(targetWrapped, currentOutput);
        double servoTarget = nearest * INV_GEAR_RATIO;
        for (int i = 0; i < servos.size(); i++) lastSetTargetDeg.set(i, servoTarget);
        servosBusy = true;
    }

    public void moveServosByRotation(double deltaOutputDeg) {
        double currentServo = getContinuousServoAngleDeg();
        double newTarget = currentServo + deltaOutputDeg * INV_GEAR_RATIO;
        for (int i = 0; i < servos.size(); i++) lastSetTargetDeg.set(i, newTarget);
        servosBusy = true;
    }

    // --- Update loop ---
    public void update() {
        boolean anyBusy = false;
        double currentServo = getContinuousServoAngleDeg();

        for (int i = 0; i < servos.size(); i++) {
            double target = lastSetTargetDeg.get(i);
            double error = target - currentServo;
            error = Range.clip(error, -180.0, 180.0);
            double absErr = Math.abs(error);

            // Derivative
            double derivative = error - lastError;
            lastError = error;

            // PD output
            double pwr = kP * error + kD * derivative;
            if (Math.abs(pwr) < minPower) pwr = Math.signum(pwr) * minPower;
            pwr = Range.clip(pwr, -1, 1);

            if (absErr > TOLERANCE_DEG) {
                // Still moving toward target
                servos.get(i).setPower(pwr * dirSign.get(i));
                anyBusy = true;
            } else {
                // Within tolerance → hold position firmly
                servos.get(i).setPower(pwr * dirSign.get(i));
            }
        }

        servosBusy = anyBusy; // only true while approaching, but holding always continues
    }

    // --- Status ---
    public boolean isServosBusy() { return servosBusy; }
    public double getContinuousAngleDeg() { return getContinuousServoAngleDeg() * GEAR_RATIO; }

    // --- Encoder logic ---
    private double rawAngleDeg() { return (encoder.getVoltage() / VREF) * 360.0; }

    private double zeroedServoAngleDeg() {
        double raw = rawAngleDeg();
        double z = raw - zeroOffsetDeg;
        z %= 360.0;
        if (z < 0) z += 360.0;
        return z;
    }

    private double getContinuousServoAngleDeg() {
        double currZeroed = zeroedServoAngleDeg();
        double diff = currZeroed - lastZeroedDeg;
        if (diff < -WRAP_THRESH_DEG) revolutions++;
        else if (diff > WRAP_THRESH_DEG) revolutions--;
        lastZeroedDeg = currZeroed;
        return revolutions * 360.0 + currZeroed;
    }

    private double getContinuousOutputAngleDeg() {
        return getContinuousServoAngleDeg() * GEAR_RATIO;
    }

    // --- Utility ---
    private static double wrapTo0_360(double a) {
        double r = a % 360.0;
        if (r < 0) r += 360.0;
        return r;
    }

    private static double nearestEquivalent(double target0to360, double currentContinuous) {
        double k = Math.rint((currentContinuous - target0to360) / 360.0);
        return target0to360 + 360.0 * k;
    }

    public void stopAll() {
        for (CRServo s : servos) s.setPower(0);
        servosBusy = false;
    }
}