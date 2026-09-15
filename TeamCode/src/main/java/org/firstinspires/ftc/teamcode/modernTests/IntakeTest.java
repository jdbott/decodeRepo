package org.firstinspires.ftc.teamcode.modernTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Intake Test", group = "Testing")
public class IntakeTest extends LinearOpMode {

    private static final double STICK_DEADBAND = 0.02;

    // Shooter encoder: ticks per motor revolution (bare-motor encoder count; adjust for your motor)
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_RAD_PER_TICK = 2.0 * Math.PI / SHOOTER_TICKS_PER_REV;
    private static final double SHOOTER_VELOCITY_INCREMENT_RAD_PER_SEC = 5.0;

    // Velocity PID + feedforward gains, live-tunable from FTC Dashboard
    public static double kP = 0.003;
    public static double kV = 0.0016;
    public static double kS = 0.05;

    private static double applyDeadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }

    @Override
    public void runOpMode() {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Servo gate = hardwareMap.get(Servo.class, "gate");
        boolean gateOpen = false;
        boolean lastRightBumper = false;
        boolean intakeRunning = true;
        boolean lastLeftBumper = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        gate.setPosition(0.55);

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Reverse the right side so positive power drives both sides forward
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        intake.setPower(1.0);
        double shooterTargetVelocity = 300.0; // rad/s

        while (opModeIsActive()) {
            boolean rightBumper = gamepad1.right_bumper;
            if (rightBumper && !lastRightBumper) {
                gateOpen = !gateOpen;
                gate.setPosition(gateOpen ? 0.4 : 0.55);
            }
            lastRightBumper = rightBumper;

            boolean leftBumper = gamepad1.left_bumper;
            if (leftBumper && !lastLeftBumper) {
                intakeRunning = !intakeRunning;
                intake.setPower(intakeRunning ? 1.0 : 0.0);
            }
            lastLeftBumper = leftBumper;

            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp) {
                shooterTargetVelocity += SHOOTER_VELOCITY_INCREMENT_RAD_PER_SEC;
            }
            lastDpadUp = dpadUp;

            boolean dpadDown = gamepad1.dpad_down;
            if (dpadDown && !lastDpadDown) {
                shooterTargetVelocity = Math.max(0.0, shooterTargetVelocity - SHOOTER_VELOCITY_INCREMENT_RAD_PER_SEC);
            }
            lastDpadDown = dpadDown;

            double shooterVelocity = Math.abs(shooter.getVelocity()) * SHOOTER_RAD_PER_TICK;
            double shooterError = shooterTargetVelocity - shooterVelocity;
            double shooterFeedforward = kV * shooterTargetVelocity + kS;
            double shooterPower = Range.clip(shooterFeedforward + kP * shooterError, 0.0, 1.0);
            shooter.setPower(shooterPower);

            double y = applyDeadband(-gamepad1.left_stick_y);
            double x = applyDeadband(gamepad1.left_stick_x);
            double rx = applyDeadband(gamepad1.right_stick_x);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Shooter Target Velocity (rad/s)", shooterTargetVelocity);
            telemetry.addData("Shooter Velocity (rad/s)", shooterVelocity);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Left Front", leftFrontPower);
            telemetry.addData("Left Back", leftBackPower);
            telemetry.addData("Right Front", rightFrontPower);
            telemetry.addData("Right Back", rightBackPower);
            telemetry.update();
        }

        intake.setPower(0);
        shooter.setPower(0);
    }
}
