package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Pivot {
    private static final double TICKS_PER_REV = 1993.6;   // motor ticks per rev (output shaft)
    private static final double GEAR_REDUCTION = 1.8;     // 60:108 reduction
    private static final double TICKS_PER_OUTPUT_REV = TICKS_PER_REV * GEAR_REDUCTION;

    // Keep your existing proportional gains
    private static final double KP_UP = 0.03;             // same as before
    private static final double KP_CORRECT = 0.04;        // same as before

    // New: tame “with gravity” motion
    private static final double KP_DOWN = 0.02;           // gentler proportional gain when going down
    private static final double MAX_POWER_UP = 1.0;       // unchanged
    private static final double MAX_POWER_DOWN = 0.5;    // cap downward power
    private static final double SOFT_LAND_ANGLE_DEG = 20; // last degrees near bottom
    private static final double MAX_POWER_SOFT_LAND = 0.1;

    // Optional: keep the commanded range conservative (tune as you like)
    private static final double MIN_ANGLE_DEG = 0.0;
    private static final double MAX_ANGLE_DEG = 80.0;

    private final DcMotorEx motor;
    double lastSetPivotAngle = 0;
    boolean pivotMotorBusy = false;

    public Pivot(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "pivotMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void movePivotToAngle(double targetAngle) {
        // Keep your previous behavior but clip into a safe range
        lastSetPivotAngle = Range.clip(targetAngle, MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        double current = getPivotAngle();
        double error = lastSetPivotAngle - current;

        // If far away, drive. Otherwise stop and mark idle.
        if (Math.abs(error) > 20) {
            double power;

            if (error > 0) {
                // Going UP against gravity: unchanged PID and caps
                power = KP_UP * error;
                power = Range.clip(power, -MAX_POWER_UP, MAX_POWER_UP);
            } else {
                // Going DOWN with gravity: use smaller gain and lower caps
                power = KP_DOWN * error; // error is negative here

                // Soft-landing zone near the bottom
                double downCap = (current <= SOFT_LAND_ANGLE_DEG) ? MAX_POWER_SOFT_LAND : MAX_POWER_DOWN;

                // Because error < 0, cap the magnitude toward -downCap
                power = Range.clip(power, -downCap, downCap);
            }

            motor.setPower(power);
            pivotMotorBusy = true;
        } else {
            motor.setPower(0);
            pivotMotorBusy = false;
        }
    }

    public void correctPivotPosition() {
        if (!pivotMotorBusy) {
            double error = lastSetPivotAngle - getPivotAngle();
            // Keep your existing correctional PID exactly the same
            double power = Range.clip(KP_CORRECT * error, -1, 1);

            // Apply the same downward damping when correcting with gravity to avoid micro-slams
            if (error < 0) {
                double current = getPivotAngle();
                double downCap = (current <= SOFT_LAND_ANGLE_DEG) ? MAX_POWER_SOFT_LAND : MAX_POWER_DOWN;
                power = Range.clip(power, -downCap, downCap);
            }

            motor.setPower(power);
        }
    }

    public void update() {
        if (isPivotMotorBusy()) {
            movePivotToAngle(lastSetPivotAngle);
        } else {
            correctPivotPosition();
        }
    }

    public boolean isPivotMotorBusy() {
        return pivotMotorBusy;
    }

    public double getPivotAngle() {
        double currentTicks = motor.getCurrentPosition();
        return (currentTicks / TICKS_PER_OUTPUT_REV) * 360.0;
    }

    public void zeroPivot() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getPower() {
        return motor.getPower();
    }
}