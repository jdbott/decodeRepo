package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled

@TeleOp(name = "Simple Mecanum TeleOp")
public class PPDTest extends LinearOpMode {

    // Hardware names must match your RC config:
    // leftF, leftB, rightF, rightB
    private DcMotorEx leftF, leftB, rightF, rightB;

    @Override
    public void runOpMode() {
        // Map
        leftF  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftB  = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightF = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightB = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Basic setup
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse one side so forward stick = forward robot
        leftF.setDirection(DcMotorSimple.Direction.FORWARD);
        leftB.setDirection(DcMotorSimple.Direction.FORWARD);
        rightF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightB.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready. Press PLAY.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Left stick: translation; Right stick X: rotation
            double y  =  gamepad1.left_stick_y;   // forward/back (up is negative, so invert)
            double x  =  -gamepad1.left_stick_x;   // strafe
            double rx =  -gamepad1.right_stick_x;  // rotate

            // Sum of vectors -> raw wheel powers
            double lf = y + x + rx;
            double lb = y - x + rx;
            double rf = y - x - rx;
            double rb = y + x - rx;

            // Proportional scaling if anything exceeds |1|
            double max = Math.max(
                    1.0,
                    Math.max(Math.abs(lf),
                            Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb))))
            );

            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;

            // Apply
            leftF.setPower(lf);
            leftB.setPower(lb);
            rightF.setPower(rf);
            rightB.setPower(rb);

            // Minimal telemetry for sanity
            telemetry.addData("y/x/rx", "%.2f / %.2f / %.2f", y, x, rx);
            telemetry.addData("LF/LB/RF/RB", "%.2f / %.2f / %.2f / %.2f", lf, lb, rf, rb);
            telemetry.update();
        }
    }
}