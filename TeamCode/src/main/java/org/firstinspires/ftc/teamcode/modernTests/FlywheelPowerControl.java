package org.firstinspires.ftc.teamcode.modernTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
@Disabled

@TeleOp(name = "Flywheel Power Control")
public class FlywheelPowerControl extends LinearOpMode {

    // Declare motors
    private DcMotor motorOne;
    private DcMotor motorTwo;

    // Power variable
    private double flywheelPower = 0.0;

    @Override
    public void runOpMode() {
        // Initialize motors
        motorOne = hardwareMap.get(DcMotor.class, "motor1");
        motorTwo = hardwareMap.get(DcMotor.class, "motor2");

        // Reverse motor directions
        motorOne.setDirection(DcMotor.Direction.REVERSE);
        motorTwo.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Debounce control
        boolean dpadUpPressedLast = false;
        boolean dpadDownPressedLast = false;

        while (opModeIsActive()) {

            // Increase power
            if (gamepad1.dpad_up && !dpadUpPressedLast) {
                flywheelPower += 0.1;
            }

            // Decrease power
            if (gamepad1.dpad_down && !dpadDownPressedLast) {
                flywheelPower -= 0.1;
            }

            // Update button state
            dpadUpPressedLast = gamepad1.dpad_up;
            dpadDownPressedLast = gamepad1.dpad_down;

            // Clip power between 0 and 1
            flywheelPower = Range.clip(flywheelPower, 0.0, 1.0);

            // Set motor powers
            motorOne.setPower(flywheelPower);
            motorTwo.setPower(flywheelPower);

            // Telemetry
            telemetry.addData("Flywheel Power", "%.1f", flywheelPower);
            telemetry.update();

            sleep(100); // Small delay to debounce button press
        }
    }
}