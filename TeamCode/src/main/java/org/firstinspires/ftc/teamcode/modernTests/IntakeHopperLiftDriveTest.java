package org.firstinspires.ftc.teamcode.modernTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Combines {@link IntakeHopperTest}, {@link IntakeTest}, and {@link LiftTest} into one
 * driveable test opmode.
 *
 * Controls:
 *   Left stick / right stick  - robot-centric mecanum drive (same deadband as IntakeTest)
 *   Left bumper               - toggle intake on/off
 *   Right bumper               - toggle hopper on/off
 *   D-pad up                  - lift to 24 in
 *   D-pad down                - lift to 0 in
 */
@TeleOp(name = "Intake Hopper Lift Drive Test", group = "Testing")
public class IntakeHopperLiftDriveTest extends LinearOpMode {

    private static final double STICK_DEADBAND = 0.02;

    private static double applyDeadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }

    private static final double INTAKE_POWER = 1.0;
    private static final double HOPPER_POWER = 0.75;

    // goBILDA 5000-series bare motor: 28 ticks/rev at the motor shaft
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    // External reduction: 13-tooth pinion drives 80-tooth hub gear (output slower than motor)
    private static final double PINION_TEETH = 13.0;
    private static final double HUB_TEETH = 80.0;
    private static final double OUTPUT_TICKS_PER_REV = MOTOR_TICKS_PER_REV * (HUB_TEETH / PINION_TEETH);

    // 60-tooth GT2 pulley (2mm pitch) on the same shaft as the hub gear
    private static final double PULLEY_TEETH = 60.0;
    private static final double GT2_PITCH_MM = 2.0;
    private static final double IN_PER_MM = 1.0 / 25.4;
    private static final double INCHES_PER_OUTPUT_REV = (PULLEY_TEETH * GT2_PITCH_MM) * IN_PER_MM;

    private static final double TICKS_PER_INCH = OUTPUT_TICKS_PER_REV / INCHES_PER_OUTPUT_REV;

    private static final double LIFT_MIN_INCHES = 0;
    private static final double LIFT_MAX_INCHES = 13.5;

    private static final double kP = 0.02;

    @Override
    public void runOpMode() {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor hopper = hardwareMap.get(DcMotor.class, "hoper");

        DcMotor lift = hardwareMap.get(DcMotor.class, "lift1");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        boolean intakeOn = false;
        boolean hopperOn = false;
        boolean leftBumperWasPressed = false;
        boolean rightBumperWasPressed = false;
        double targetInches = LIFT_MIN_INCHES;

        waitForStart();

        while (opModeIsActive()) {
            // --- Robot-centric mecanum drive ---
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

            // --- Intake toggle (left bumper) ---
            if (gamepad1.left_bumper && !leftBumperWasPressed) {
                intakeOn = !intakeOn;
            }
            leftBumperWasPressed = gamepad1.left_bumper;
            intake.setPower(intakeOn ? INTAKE_POWER : 0);

            // --- Hopper toggle (right bumper) ---
            if (gamepad1.right_bumper && !rightBumperWasPressed) {
                hopperOn = !hopperOn;
            }
            rightBumperWasPressed = gamepad1.right_bumper;
            hopper.setPower(hopperOn ? HOPPER_POWER : 0);

            // --- Lift position (D-pad up/down) ---
            if (gamepad1.dpad_up) {
                targetInches = LIFT_MAX_INCHES;
            } else if (gamepad1.dpad_down) {
                targetInches = LIFT_MIN_INCHES;
            }
            targetInches = Math.max(LIFT_MIN_INCHES, Math.min(LIFT_MAX_INCHES, targetInches));

            int targetTicks = (int) Math.round(targetInches * TICKS_PER_INCH);
            int currentTicks = lift.getCurrentPosition();
            int error = targetTicks - currentTicks;

            double liftPower = kP * error;
            liftPower = Math.max(-1.0, Math.min(1.0, liftPower));
            lift.setPower(liftPower);

            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Hopper", hopperOn ? "ON" : "OFF");
            telemetry.addData("Lift Target (in)", targetInches);
            telemetry.addData("Lift Current (in)", currentTicks / TICKS_PER_INCH);
            telemetry.addData("Left Front", leftFrontPower);
            telemetry.addData("Left Back", leftBackPower);
            telemetry.addData("Right Front", rightFrontPower);
            telemetry.addData("Right Back", rightBackPower);
            telemetry.update();
        }

        intake.setPower(0);
        hopper.setPower(0);
        lift.setPower(0);
    }
}
