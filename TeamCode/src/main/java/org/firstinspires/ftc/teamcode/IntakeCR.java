package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "IntakeCRTest", group = "Tests")
public class IntakeCR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- CRServo setup ---
        ServoController servoController = new ServoController(
                hardwareMap,
                new String[]{"CRServo1", "CRServo2"},
                new double[]{+1.0, -1.0},
                "CRServo1A"
        );

        // --- Intake motor setup ---
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Variables ---
        double intakePower = 1.0;
        double incrementDeg = 120.0;

        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastA = false;

        telemetry.addLine("Initialized. Press START to begin.");
        telemetry.update();

        waitForStart();
        servoController.zeroNow();

        telemetry.addLine("Zeroed encoder position.");
        telemetry.update();

        while (opModeIsActive()) {
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean aButton = gamepad1.a;

            // --- Servo controls (same as CRServoTest) ---
            if (dpadLeft && !lastDpadLeft) {
                servoController.moveServosByRotation(-incrementDeg);
            } else if (dpadRight && !lastDpadRight) {
                servoController.moveServosByRotation(incrementDeg);
            }

            servoController.update();

            // --- Intake controls ---
            if (gamepad1.y) {
                intakeMotor.setPower(intakePower); // forward
            } else if (gamepad1.b) {
                intakeMotor.setPower(-intakePower); // reverse
            } else if (gamepad1.x) {
                intakeMotor.setPower(0); // stop
            }

            // --- Telemetry ---
            telemetry.addData("Servo Busy", servoController.isServosBusy());
            telemetry.addData("Servo Angle (deg)", "%.2f", servoController.getContinuousAngleDeg());
            telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
            telemetry.update();

            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;
            lastA = aButton;

            sleep(20);
        }

        servoController.stopAll();
        intakeMotor.setPower(0);
    }
}