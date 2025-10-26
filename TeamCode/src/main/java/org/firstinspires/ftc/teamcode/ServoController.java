package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ServoController {

    // --- Hardware ---
    private CRServo servo1, servo2;
    private AnalogInput encoder1;  // only first servo has feedback

    // --- Encoder tracking ---
    private double zeroOffsetDeg = 0.0;
    private double lastZeroedDeg = 0.0;
    private int revolutions = 0;

    // --- Control parameters ---
    private double kP = 0.006;
    private double kI = 0.0005;        // small integral gain
    private double integral = 0.0;
    private double lastTargetDeg = 0.0;
    private boolean servosBusy = false;

    // --- Constants ---
    private static final double VREF = 3.3;
    private static final double GEAR_RATIO = 30.0 / 36.0;
    private static final double INV_GEAR_RATIO = 1.0 / GEAR_RATIO;
    private static final double MAX_OUTPUT = 0.9;   // limit command to 90%

    // --- Target tracking ---
    private double lastSetTargetDeg = 0.0;

    // --- Constructor ---
    public ServoController(HardwareMap hw) {
        servo1 = hw.get(CRServo.class, "CRServo1");
        servo2 = hw.get(CRServo.class, "CRServo2");
        servo2.setDirection(DcMotorSimple.Direction.REVERSE); // ensure same motion
        encoder1 = hw.get(AnalogInput.class, "CRServo1A");
        zeroNow();
    }

    // --- Configuration ---
    public void setKP(double gain) { kP = gain; }
    public void setKI(double gain) { kI = gain; }

    // --- Zero encoder ---
    public void zeroNow() {
        zeroOffsetDeg = rawAngleDeg();
        lastZeroedDeg = 0.0;
        revolutions = 0;
        integral = 0;
        servosBusy = false;
        stopAll();
    }

    // --- Command absolute ---
    // --- Command absolute ---
    public void moveServosToPosition(double targetOutputDeg) {
        // Determine direction of motion based on current vs target
        double currentOutputDeg = getContinuousAngleDeg1();
        double offset = 0.0;

        // Apply bidirectional offset (tune value if needed)
        double OFFSET_MAGNITUDE = 10;
        if (targetOutputDeg > currentOutputDeg) {
            offset = -OFFSET_MAGNITUDE;  // moving forward, overshoots
        } else if (targetOutputDeg < currentOutputDeg) {
            offset = offset;   // moving backward, undershoots
        }

        // Apply direction-based offset
        targetOutputDeg += offset;

        double targetServoWrapped = wrap0_360(targetOutputDeg * INV_GEAR_RATIO);
        double currentServo = getContinuousOutputAngleDeg();
        double nearestServo = nearestEquivalent(targetServoWrapped, currentServo);
        lastSetTargetDeg = nearestServo;
        servosBusy = true;
    }


    // --- Command relative (still available) ---
    public void moveServosByRotation(double deltaOutputDeg) {
        double currentServo = getContinuousOutputAngleDeg();
        double newTarget = currentServo + deltaOutputDeg * INV_GEAR_RATIO;
        lastSetTargetDeg = newTarget;
        servosBusy = true;
    }

    // --- Main update loop ---
    public void update() {
        double currentServo = getContinuousOutputAngleDeg();
        double targetServo = lastSetTargetDeg;

        // Reset integral if target changed
        if (targetServo != lastTargetDeg) {
            integral = 0;
            lastTargetDeg = targetServo;
        }

        double error = shortestDiffDeg(targetServo, currentServo);

        // Integrate with anti-windup and decay
        integral += error * 0.02;                   // assuming 20 ms loop
        integral = Range.clip(integral, -100, 100); // clamp
        integral *= 0.98;                           // slow decay

        double cmd = kP * error + kI * integral;
        cmd = Range.clip(cmd, -MAX_OUTPUT, MAX_OUTPUT);

        servo1.setPower(cmd);
        servo2.setPower(cmd);

        servosBusy = Math.abs(error) > 1.0;
    }

    // --- Status ---
    public boolean isServosBusy() { return servosBusy; }
    public double getContinuousAngleDeg1() { return getContinuousOutputAngleDeg() * GEAR_RATIO; }

    // --- Encoder math ---
    private double rawAngleDeg() {
        return (encoder1.getVoltage() / VREF) * 360.0;
    }

    private double zeroedServoAngleDeg() {
        double z = (rawAngleDeg() - zeroOffsetDeg) % 360.0;
        if (z < 0) z += 360.0;
        return z;
    }

    public double getContinuousOutputAngleDeg() {
        double curr = zeroedServoAngleDeg();
        double diff = curr - lastZeroedDeg;

        if (diff > 180) diff -= 360;
        else if (diff <= -180) diff += 360;

        double next = lastZeroedDeg + diff;

        if (next >= 360) {
            next -= 360;
            revolutions++;
        } else if (next < 0) {
            next += 360;
            revolutions--;
        }

        lastZeroedDeg = next;
        return revolutions * 360.0 + lastZeroedDeg;
    }

    // --- Helpers ---
    private static double wrap0_360(double a) {
        double r = a % 360.0;
        if (r < 0) r += 360.0;
        return r;
    }

    private static double shortestDiffDeg(double target, double current) {
        double d = target - current;
        if (d > 180) d -= 360;
        else if (d <= -180) d += 360;
        return d;
    }

    private static double nearestEquivalent(double target0to360, double currentContinuous) {
        double k = Math.rint((currentContinuous - target0to360) / 360.0);
        return target0to360 + 360.0 * k;
    }

    // --- Stop ---
    public void stopAll() {
        servo1.setPower(0);
        servo2.setPower(0);
        servosBusy = false;
    }
}