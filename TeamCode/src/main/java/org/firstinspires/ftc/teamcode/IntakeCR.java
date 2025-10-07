package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "IntakeCRTest", group = "Tests")
public class IntakeCR extends LinearOpMode {

    private Servo popperServo;

    // Shooter instance
    private Shooter shooter;

    // Power variables
    private double wheelVelocity = 0.0;
    private double wheelRPM = 0.0;
    private boolean useRPM = false;

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

        popperServo = hardwareMap.get(Servo.class, "popperServo");

        String[] motorNames = {"motor1", "motor2"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE};
        DcMotorEx.RunMode[] runModes = {DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, DcMotorEx.RunMode.RUN_USING_ENCODER};
        shooter = new Shooter();
        shooter.init(hardwareMap, "motor1", "motor2", DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);


        // --- Variables ---
        double intakePower = 1.0;
        double incrementDeg = 120.0;

        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastA = false;


        // Debounce control
        boolean dpadUpPressedLast = false;
        boolean dpadDownPressedLast = false;
        boolean aPressedLast = false;

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

            if (gamepad1.right_bumper) {
                servoController.moveServosToPosition(60);
            } else if (gamepad1.left_bumper) {
                servoController.moveServosToPosition(0);
            }

            if (gamepad1.right_trigger > 0.5) {
                popperServo.setPosition(0.45);
            } else {
                popperServo.setPosition(0.14);
            }

            if (gamepad1.right_stick_button && !dpadUpPressedLast) {
                wheelRPM += 100.0; // Increase by 100 RPM
                wheelRPM = Range.clip(wheelRPM, 0.0, 6000.0);
                telemetry.addLine("Right stick pressed. Increasing RPM to " + wheelRPM + " RPM.");
            }
            if (gamepad1.left_stick_button && !dpadDownPressedLast) {
                wheelRPM -= 100.0; // Decrease by 100 RPM
                wheelRPM = Range.clip(wheelRPM, 0.0, 6000.0);
                telemetry.addLine("Left stick pressed. Increasing RPM to " + wheelRPM + " RPM.");
            }
            shooter.setTargetRPM(wheelRPM);
            shooter.update();

            // Update last button states
            dpadUpPressedLast = gamepad1.right_stick_button;
            dpadDownPressedLast = gamepad1.left_stick_button;

            // --- Telemetry ---
            telemetry.addData("Servo Busy", servoController.isServosBusy());
            // telemetry.addData("Current RPM", shooter.getCurrentRPM());
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