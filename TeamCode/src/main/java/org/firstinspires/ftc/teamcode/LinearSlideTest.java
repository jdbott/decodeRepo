
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp (name = "Slide Test")
public class LinearSlideTest extends LinearOpMode {
    private LinearSlide linearSlide;

    @Override
    public void runOpMode() {
        String[] motorNames = {"slides"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE};
        linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 28); // Example ticksPerInch and limits

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
                linearSlide.moveSlidesToPositionInches(28);
            }

            linearSlide.update();

            telemetry.addData("Position Inches", linearSlide.slidesPositionInches());
            telemetry.addData("Is Busy", linearSlide.isSlideMotorsBusy());
            telemetry.update();
        }
    }
}