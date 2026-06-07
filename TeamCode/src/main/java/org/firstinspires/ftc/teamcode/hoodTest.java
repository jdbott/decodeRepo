package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Servo Adjust", group = "Test")
public class hoodTest extends LinearOpMode {

    private Servo hoodServo;

    // Starting position
    private double servoPosition = 0.5;

    // Amount to change each button press
    private static final double STEP = 0.01;

    // For dpad debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {

        // Hardware name in configuration must be exactly: HoodServo
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Set servo to 1 during init
        hoodServo.setPosition(servoPosition);

        // Init loop
        while (opModeInInit()) {
            hoodServo.setPosition(servoPosition);

            telemetry.addLine("Initialized");
            telemetry.addData("Servo Name", "HoodServo");
            telemetry.addData("Current Position", "%.2f", servoPosition);
            telemetry.addLine("Press play to start");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            // D-pad up: increase once per press
            if (currentDpadUp && !lastDpadUp) {
                servoPosition += STEP;
            }

            // D-pad down: decrease once per press
            if (currentDpadDown && !lastDpadDown) {
                servoPosition -= STEP;
            }

            // Clamp between 0 and 1
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

            // Apply servo position
            hoodServo.setPosition(servoPosition);

            // Save button states for debounce
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            telemetry.addLine("TeleOp Running");
            telemetry.addData("Servo Name", "HoodServo");
            telemetry.addData("Current Position", "%.2f", servoPosition);
            telemetry.addLine("D-pad Up = +0.01");
            telemetry.addLine("D-pad Down = -0.01");
            telemetry.update(); // 0.42, 0.95

            sleep(20);
        }
    }
}