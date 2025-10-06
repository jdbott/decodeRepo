package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class ServoController {
    // Hardware
    private final List<CRServo> servos = new ArrayList<>();
    private final AnalogInput encoder; // single shared encoder

    // Shared encoder state
    private double zeroOffsetDeg = 0.0;
    private double lastZeroedDeg = 0.0;
    private int revolutions = 0;

    // Per-servo configuration
    private final List<Double> dirSign = new ArrayList<>();
    private final List<Double> lastSetTargetDeg = new ArrayList<>();

    // Tuning
    private double kP = 0.01;
    private double kD = 0.002;      // <-- Derivative term added
    private double lastError = 0.0; // <-- To compute derivative
    private double minPower = 0.0;
    private boolean servosBusy = false;

    // Constants
    private static final double TOLERANCE_DEG = 4.0;
    private static final double VREF = 3.3;
    private static final double WRAP_THRESH_DEG = 270.0;

    // Gear ratio constants (servo gear : output gear = 30 : 36)
    private static final double GEAR_RATIO = 30.0 / 36.0;
    private static final double INV_GEAR_RATIO = 1.0 / GEAR_RATIO;

    /**
     * Constructor.
     * @param hw Hardware map.
     * @param servoNames Array of servo names.
     * @param directions Array of direction multipliers (+1 or -1) for each servo.
     * @param encoderName Analog input name for the encoder.
     */
    public ServoController(HardwareMap hw, String[] servoNames, double[] directions, String encoderName) {
        if (servoNames.length != directions.length)
            throw new IllegalArgumentException("Servo names and directions arrays must have the same length.");

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
    public void setKD(double gain) { kD = gain; } // <-- Setter for derivative
    public void setMinPower(double pwr) { minPower = Math.abs(pwr); }
    public void setServoReversed(int i, boolean reversed) { dirSign.set(i, reversed ? -1.0 : 1.0); }

    // Zero shared encoder
    public void zeroNow() {
        double raw = rawAngleDeg();
        zeroOffsetDeg = raw;
        lastZeroedDeg = 0.0;
        revolutions = 0;
        for (CRServo s : servos) s.setPower(0);
        servosBusy = false;
        lastError = 0.0; // reset derivative memory
    }

    // --- Commands ---
    public void moveServosToPosition(double targetOutputDeg) {
        double t0_360 = wrapTo0_360(targetOutputDeg);
        double currentOutput = getContinuousOutputAngleDeg();
        double nearest = nearestEquivalent(t0_360, currentOutput);
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
            double error = lastSetTargetDeg.get(i) - currentServo;
            error = Range.clip(error, -180.0, 180.0);
            double absErr = Math.abs(error);

            // Derivative term: change in error per cycle
            double derivative = error - lastError;
            lastError = error;

            if (absErr > TOLERANCE_DEG) {
                double pwr = kP * error + kD * derivative; // <-- PD control
                if (Math.abs(pwr) < minPower) pwr = Math.signum(pwr) * minPower;
                pwr = Range.clip(pwr, -1, 1);
                servos.get(i).setPower(pwr * dirSign.get(i));
                anyBusy = true;
            } else {
                servos.get(i).setPower(0);
            }
        }
        servosBusy = anyBusy;
    }

    // --- Status ---
    public boolean isServosBusy() { return servosBusy; }
    public double getContinuousAngleDeg() { return getContinuousServoAngleDeg() * GEAR_RATIO; }

    // --- Encoder tracking ---
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

    // Stop all
    public void stopAll() {
        for (CRServo s : servos) s.setPower(0);
        servosBusy = false;
    }
}