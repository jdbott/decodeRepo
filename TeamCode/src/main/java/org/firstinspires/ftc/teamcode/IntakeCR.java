package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "IntakeCRTest", group = "Tests")
public class IntakeCR extends LinearOpMode {

    private Servo popperServo;
    private Servo gateServo;
    private Shooter shooter;

    // --- FSM variables ---
    private enum GateState { IDLE, WAITING, RAISING }
    private GateState gateState = GateState.IDLE;
    private long gateTimer = 0;
    private boolean gateRequested = false;
    private boolean servoMovePending = false;
    private double moveAmountDeg = 0;
    private boolean useAbsolute = false; // flag to distinguish position vs rotation
    private double absoluteTarget = 0.0;

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
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // --- Shooter setup ---
        shooter = new Shooter();
        shooter.init(hardwareMap, "motor1", "motor2",
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);

        // --- Variables ---
        double intakePower = 1.0;
        double incrementDeg = 120.0;
        double wheelRPM = 0.0;

        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastRightStick = false;
        boolean lastLeftStick = false;
        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;

        telemetry.addLine("Initialized. Press START to begin.");
        telemetry.update();

        servoController.zeroNow();
        gateServo.setPosition(0.48);

        waitForStart();

        while (opModeIsActive()) {
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean rightStick = gamepad1.right_stick_button;
            boolean leftStick = gamepad1.left_stick_button;
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;

            // --- Revolver rotation commands ---
            if (dpadLeft && !lastDpadLeft) requestGateBeforeMove(-incrementDeg, false, 0);
            if (dpadRight && !lastDpadRight) requestGateBeforeMove(incrementDeg, false, 0);

            // --- Absolute servo position toggles ---
            if (rightBumper && !lastRightBumper) requestGateBeforeMove(0, true, 60);
            if (leftBumper && !lastLeftBumper) requestGateBeforeMove(0, true, 0);

            // --- Intake control ---
            if (gamepad1.y) intakeMotor.setPower(intakePower);
            else if (gamepad1.b) intakeMotor.setPower(-intakePower);
            else if (gamepad1.x) intakeMotor.setPower(0);

            // --- Popper control ---
            if (gamepad1.right_trigger > 0.5) popperServo.setPosition(0.45);
            else popperServo.setPosition(0.14);

            // --- Shooter RPM control ---
            if (rightStick && !lastRightStick)
                wheelRPM = Range.clip(wheelRPM + 250, 0.0, 4900);
            if (leftStick && !lastLeftStick)
                wheelRPM = Range.clip(wheelRPM - 250, 0.0, 4900);

            shooter.setTargetRPM(wheelRPM);
            shooter.update();

            // --- Gate FSM update ---
            updateGateFSM(servoController);

            // --- Telemetry ---
            telemetry.clearAll();
            telemetry.addLine("=== System Status ===");
            telemetry.addData("Gate State", gateState);
            telemetry.addData("Servo Angle (deg)", "%.2f", servoController.getContinuousAngleDeg());
            telemetry.addData("Shooter Target RPM", "%.0f", wheelRPM);
            telemetry.addData("Shooter Current RPM", "%.0f", shooter.getMasterRPM());
            telemetry.addData("Gate Pos", "%.2f", gateServo.getPosition());
            telemetry.update();

            // --- State memory ---
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;
            lastRightStick = rightStick;
            lastLeftStick = leftStick;
            lastRightBumper = rightBumper;
            lastLeftBumper = leftBumper;

            sleep(20);
        }

        servoController.stopAll();
        intakeMotor.setPower(0);
    }

    // ---------------- Gate FSM Methods ----------------
    private void requestGateBeforeMove(double moveDeg, boolean absolute, double absTarget) {
        gateRequested = true;
        moveAmountDeg = moveDeg;
        useAbsolute = absolute;
        absoluteTarget = absTarget;
    }

    private void updateGateFSM(ServoController servoController) {
        double gateDownPos = 0.38;
        double gateUpPos = 0.48;
        double delaySec = 0.08; // adjustable delay before rotation
        long now = System.currentTimeMillis();

        switch (gateState) {
            case IDLE:
                if (gateRequested) {
                    gateRequested = false;
                    gateServo.setPosition(gateDownPos);
                    gateTimer = now + (long) (delaySec * 1000);
                    servoMovePending = true;
                    gateState = GateState.WAITING;
                }
                break;

            case WAITING:
                if (now >= gateTimer) {
                    if (servoMovePending) {
                        if (useAbsolute) {
                            servoController.moveServosToPosition(absoluteTarget);
                        } else {
                            servoController.moveServosByRotation(moveAmountDeg);
                        }
                        servoMovePending = false;
                        gateTimer = now + 400; // short pause before raising
                        gateState = GateState.RAISING;
                    }
                }
                break;

            case RAISING:
                if (now >= gateTimer) {
                    gateServo.setPosition(gateUpPos);
                    gateState = GateState.IDLE;
                }
                break;
        }

        // Always update servo controller
        servoController.update();
    }
}