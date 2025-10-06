package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CRServoTest", group = "Tests")
public class CRServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Example:
        // Two servos, first normal (+1), second reversed (-1)
        ServoController servoController = new ServoController(
                hardwareMap,
                new String[]{"CRServo1", "CRServo2"},
                new double[]{+1.0, -1.0},  // direction for each servo
                "CRServo1A"                 // encoder is on CRServo1
        );

        telemetry.addLine("CRServoTest initialized. Press START to begin.");
        telemetry.update();

        waitForStart();
        servoController.zeroNow();
        telemetry.addLine("Zeroed current position.");
        telemetry.update();

        double incrementDeg = 120.0;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastA = false;

        while (opModeIsActive()) {
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean aButton = gamepad1.a;

            // Rotate relative increments
            if (dpadLeft && !lastDpadLeft) {
                servoController.moveServosByRotation(-incrementDeg);
            } else if (dpadRight && !lastDpadRight) {
                servoController.moveServosByRotation(incrementDeg);
            }

            // Toggle between 0° and 180°
            if (aButton && !lastA) {
                double currentAngle = servoController.getContinuousAngleDeg();
                double target = (Math.abs(currentAngle % 360) < 90) ? 180.0 : 0.0;
                servoController.moveServosToPosition(target);
            }

            servoController.update();

            telemetry.addData("Servo busy", servoController.isServosBusy());
            telemetry.addData("Angle (deg)", "%.2f", servoController.getContinuousAngleDeg());
            telemetry.update();

            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;
            lastA = aButton;

            sleep(20);
        }

        servoController.stopAll();
    }
}