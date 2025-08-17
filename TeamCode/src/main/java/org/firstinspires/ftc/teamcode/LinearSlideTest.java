
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Slide Test")
public class LinearSlideTest extends LinearOpMode {
    private LinearSlide linearSlide;

    private Servo armLeft;
    private Servo armRight;

    @Override
    public void runOpMode() {
        String[] motorNames = {"slides", "slides2"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 31.1071, 0, 28); // Example ticksPerInch and limits

        // Initialize servo from hardware map
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight = hardwareMap .get(Servo.class, "armRight");


        // Set position to zero
        armLeft.setPosition(0.25);
        armRight.setPosition(0.27);

        // Wait for the Play button to be pressed
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                linearSlide.moveSlidesToPositionInches(0);
            }

            if (gamepad1.dpad_right) {
                linearSlide.moveSlidesToPositionInches(15);
            }

            if (gamepad1.dpad_up) {
                linearSlide.moveSlidesToPositionInches(27);
            }

            linearSlide.update();

            telemetry.addData("Position Inches", linearSlide.slidesPositionInches());
            telemetry.addData("Is Busy", linearSlide.isSlideMotorsBusy());
            telemetry.addData("Power", linearSlide.getPower());
            telemetry.update();
        }
    }
}