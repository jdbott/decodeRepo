
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "CR Servo Test")
public class CRServoTest extends LinearOpMode {
    private LinearSlide linearSlide;
    private ServoController servoController;

    @Override
    public void runOpMode() {
        String[] servoNames = {"CRServo1"};
        String[] encoderNames = {"CRServo1A"};
        servoController = new ServoController(hardwareMap, servoNames, encoderNames);

        // Wait for the Play button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                servoController.moveServosToPosition(140);
            }

            if (gamepad1.dpad_right) {
                servoController.moveServosToPosition(0);
            }

            if (gamepad1.dpad_up) {
                servoController.moveServosByRotation(800);
            }

            servoController.update();
        }
    }
}