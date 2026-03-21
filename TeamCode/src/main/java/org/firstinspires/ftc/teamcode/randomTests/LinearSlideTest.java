
package org.firstinspires.ftc.teamcode.randomTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.hardwareClasses.Pivot;

@Disabled

@TeleOp (name = "Slide Test")
public class LinearSlideTest extends LinearOpMode {
    private LinearSlide linearSlide;

    private Servo armLeft;
    private Servo armRight;

    private Pivot pivot;

    @Override
    public void runOpMode() {
        String[] motorNames = {"slides", "slides2"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 31.1071, 0, 28); // Example ticksPerInch and limits

        // Initialize servo from hardware map
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight = hardwareMap .get(Servo.class, "armRight");

        pivot = new Pivot(hardwareMap);

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

            if (gamepad1.left_bumper) {
                pivot.movePivotToAngle(0);
            }

            if (gamepad1.right_bumper) {
                pivot.movePivotToAngle(80);
            }

            linearSlide.update();
            pivot.update();

            telemetry.addData("Slide Position Inches", linearSlide.slidesPositionInches());
            telemetry.addData("Slide Is Busy", linearSlide.isSlideMotorsBusy());
            telemetry.addData("Slide Power", linearSlide.getPower());
            telemetry.addData("Pivot Angle", pivot.getPivotAngle());
            telemetry.addData("Pivot Is Busy", pivot.isPivotMotorBusy());
            telemetry.addData("Pivot Power", pivot.getPower());
            telemetry.update();
        }
    }
}