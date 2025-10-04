package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class ServoController {
    private final List<CRServo> servos = new ArrayList<>();
    private final List<AnalogInput> encoders = new ArrayList<>();

    private double lastSetServoAngle = 0; // target angle in degrees
    private boolean servosBusy = false;
    private double kP = 0.0001;             // proportional gain
    private double minPower = 0.0;        // minimum power threshold
    private static final double TOLERANCE = 1.0; // degrees

    public ServoController(HardwareMap hardwareMap, String[] servoNames, String[] encoderNames) {
        for (String name : servoNames) {
            CRServo servo = hardwareMap.get(CRServo.class, name);
            servos.add(servo);
        }
        for (String name : encoderNames) {
            AnalogInput encoder = hardwareMap.get(AnalogInput.class, name);
            encoders.add(encoder);
        }
    }

    // Set minimum servo power threshold
    public void setMinPower(double minPower) {
        this.minPower = Math.abs(minPower);
    }

    // Convert encoder voltage to angle (0–360)
    private double getAngleFromEncoder(AnalogInput encoder) {
        return (encoder.getVoltage() / encoder.getMaxVoltage()) * 360.0;
    }

    // Move servos to absolute target angle in degrees
    public void moveServosToPosition(double targetAngle) {
        lastSetServoAngle = targetAngle;

        for (int i = 0; i < servos.size(); i++) {
            double currentAngle = getAngleFromEncoder(encoders.get(i));
            double error = wrapDegrees(targetAngle - currentAngle);

            if (Math.abs(error) > TOLERANCE) {
                double power = kP * error;

                // Apply minimum threshold
                if (minPower > 0) {
                    if (power > 0 && power < minPower) power = minPower;
                    else if (power < 0 && power > -minPower) power = -minPower;
                }

                servos.get(i).setPower(Range.clip(power, -1, 1));
                servosBusy = true;
            } else {
                servos.get(i).setPower(0);
                servosBusy = false;
            }
        }
    }

    // NEW: Move servos by a relative rotation (can exceed 360 or be negative)
    public void moveServosByRotation(double deltaAngle) {
        // Get the average current position
        double currentAngle = averageServoAngle();

        // New target is current + requested delta
        double newTarget = currentAngle + deltaAngle;

        // Save it as the target (not wrapped, allows multiple turns)
        lastSetServoAngle = newTarget;

        // Move like usual
        moveServosToPosition(lastSetServoAngle);
    }

    // Corrects positions when not busy
    public void correctServoPositions() {
        if (!servosBusy) {
            for (int i = 0; i < servos.size(); i++) {
                double currentAngle = getAngleFromEncoder(encoders.get(i));
                double error = wrapDegrees(lastSetServoAngle - currentAngle);
                double power = kP * error;
                servos.get(i).setPower(Range.clip(power, -1, 1));
            }
        }
    }

    // Updates the servo state
    public void update() {
        if (isServosBusy()) {
            moveServosToPosition(lastSetServoAngle);
        } else {
            correctServoPositions();
        }
    }

    // Check if servos are busy (based on tolerance)
    public boolean isServosBusy() {
        return servosBusy;
    }

    // Get average servo angle from encoders
    public double averageServoAngle() {
        double total = 0;
        for (AnalogInput encoder : encoders) {
            total += getAngleFromEncoder(encoder);
        }
        return total / encoders.size();
    }

    // Utility: wrap to [-180, 180)
    private double wrapDegrees(double angle) {
        return ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    // Allow tuning kP
    public void setKP(double set) {
        kP = set;
    }
}