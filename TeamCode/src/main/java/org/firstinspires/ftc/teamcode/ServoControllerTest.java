package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo Controller Test")
public class ServoControllerTest extends LinearOpMode {

    private ServoController servoController;
    private ElapsedTime debounce = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        servoController = new ServoController(hardwareMap);

        telemetry.addLine("Ready — use D-pad and A/B to test");
        telemetry.update();
        waitForStart();

        servoController.moveServosToPosition(120);

        debounce.reset();

        while (opModeIsActive()) {
            // --- Absolute positioning with D-pad ---
            if (gamepad1.dpad_left && debounce.seconds() > 0.3) {
                servoController.moveServosToPosition(0);
                debounce.reset();
            }
            if (gamepad1.dpad_up && debounce.seconds() > 0.3) {
                servoController.moveServosToPosition(120);
                debounce.reset();
            }
            if (gamepad1.dpad_right && debounce.seconds() > 0.3) {
                servoController.moveServosToPosition(240);
                debounce.reset();
            }

            // --- Relative movement with A/B ---
            if (gamepad1.a && debounce.seconds() > 0.3) {
                servoController.moveServosByRotation(-60); // move left
                debounce.reset();
            }
            if (gamepad1.b && debounce.seconds() > 0.3) {
                servoController.moveServosByRotation(60); // move right
                debounce.reset();
            }

            // --- Periodic update ---
            servoController.update();

            telemetry.addData("Target Angle (Output)", "%.2f", servoController.getTargetAngleDeg());
            telemetry.addData("Busy", servoController.isServosBusy());
            telemetry.addData("Current Angle (deg)", "%.2f", servoController.getAbsoluteAngleDeg());
            telemetry.update();

            sleep(20);
        }
    }
}