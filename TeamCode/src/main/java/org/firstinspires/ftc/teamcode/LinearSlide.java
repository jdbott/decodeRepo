package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

public class LinearSlide {
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final double maxExtensionInches;
    private final double minExtensionInches;
    private final double ticksPerInch; // Conversion factor from inches to motor ticks
    private double lastSetSlidePos = 0;
    private boolean slideMotorsBusy = false;
    private double kP = 0.005;

    // New field for minimum power threshold during extension.
    // Set this to 0 if you don't want any minimum power enforced.
    private double minExtensionPower = 0.0;

    // Constructor that accepts motor names, directions, and extension limits
    public LinearSlide(HardwareMap hardwareMap, String[] motorNames, DcMotorSimple.Direction[] directions,
                       double ticksPerInch, double minExtensionInches, double maxExtensionInches) {
        this.ticksPerInch = ticksPerInch;
        this.minExtensionInches = minExtensionInches;
        this.maxExtensionInches = maxExtensionInches;

        for (int i = 0; i < motorNames.length; i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorNames[i]);
            motor.setDirection(directions[i]);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motor);
        }
    }

    public void zeroSlides() {
        for (DcMotorEx motor : motors) { // Loop through the existing motors list
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Special method to zero the slides.
     *
     * This method performs the following:
     * 1. Drives the slide motors at full power in the negative direction.
     * 2. Waits for 0.5 seconds using opMode.sleep().
     * 3. Stops the motors, resets the encoders, and sets the current position to 0 inches.
     *
     * @param opMode A reference to the current LinearOpMode to use its sleep method.
     */
    public void specialZeroSlides(LinearOpMode opMode) {
        // Drive at full speed in negative direction.
        for (DcMotorEx motor : motors) {
            motor.setPower(-1);
        }

        // Wait 0.5 seconds using the opMode's sleep method.
        opMode.sleep(500);

        // Stop the motors and reset encoders.
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set the last known position to 0 inches.
        lastSetSlidePos = 0;
        moveSlidesToPositionInches(0);
    }

    // Get the maximum extension in inches
    public double getMaxExtensionInches() {
        return maxExtensionInches;
    }

    public double getPower() {
        return motors.get(0).getPower();
    }

    // Get the minimum extension in inches
    public double getMinExtensionInches() {
        return minExtensionInches;
    }

    // Set the minimum extension power threshold (must be non-negative).
    // For example, setMinExtensionPower(0.3) will ensure that power is never between -0.3 and 0.3.
    public void setMinExtensionPower(double minExtensionPower) {
        this.minExtensionPower = Math.abs(minExtensionPower);
    }

    // Convert inches to motor ticks
    private double inchesToTicks(double inches) {
        return inches * ticksPerInch;
    }

    // Move the slide to the target position in inches, with limits applied
    public void moveSlidesToPositionInches(double targetInches) {
        // Clamp target position within allowed range
        targetInches = Range.clip(targetInches, minExtensionInches, maxExtensionInches);

        // Convert the target position from inches to motor ticks
        double targetTicks = inchesToTicks(targetInches);

        // Move each motor to the calculated target position in ticks
        moveSlidesToPosition(targetTicks);
    }

    // Move the slides to the target position in ticks (internal method)
    private void moveSlidesToPosition(double targetTicks) {
        lastSetSlidePos = targetTicks;

        for (DcMotorEx motor : motors) {
            double error = targetTicks - motor.getCurrentPosition();

            if (Math.abs(error) > 20) { // Ensure error threshold is handled correctly
                double computedPower = error * kP;

                // Apply the minimum power threshold if needed
                if (minExtensionPower > 0) {
                    if (computedPower > 0 && computedPower < minExtensionPower) {
                        computedPower = minExtensionPower;
                    } else if (computedPower < 0 && computedPower > -minExtensionPower) {
                        computedPower = -minExtensionPower;
                    }
                }

                motor.setPower(Range.clip(computedPower, -1, 1)); // Adjust power direction
                slideMotorsBusy = true;
            } else {
                motor.setPower(0);
                slideMotorsBusy = false;
            }
        }
    }

    // Corrects the position of all motors
    public void correctSlidePositions() {
        if (!slideMotorsBusy) {
            for (DcMotorEx motor : motors) {
                double error = lastSetSlidePos - motor.getCurrentPosition();
                motor.setPower(Range.clip((error * kP), -1, 1));
            }
        }
    }

    // Updates the slide's position and correction
    public void update() {
        if (isSlideMotorsBusy()) {
            moveSlidesToPosition(lastSetSlidePos);
        } else {
            correctSlidePositions();
        }
    }

    // Checks if any motor is busy
    public boolean isSlideMotorsBusy() {
        return slideMotorsBusy;
    }

    public void setKP(double set) {
        kP = set;
    }

    // Get the average position of all motors (in inches)
    public double slidesPositionInches() {
        double totalTicks = 0;
        for (DcMotorEx motor : motors) {
            totalTicks += motor.getCurrentPosition();
        }
        return totalTicks / motors.size() / ticksPerInch; // Convert average ticks to inches
    }
}