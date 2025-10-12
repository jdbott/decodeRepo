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

    // --- Drivetrain motors ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // --- FSM variables ---
    private enum GateState { IDLE, WAITING, RAISING }
    private GateState gateState = GateState.IDLE;
    private long gateTimer = 0;
    private boolean gateRequested = false;
    private boolean servoMovePending = false;
    private double moveAmountDeg = 0;
    private boolean useAbsolute = false;
    private double absoluteTarget = 0.0;

    // --- Auto-shoot FSM ---
    private enum AutoState {
        OFF, MOVE_TO_START, WAIT_TO_SETTLE, POP_UP, POP_DOWN, REVOLVE, DONE, RETURN_TO_START
    }
    private AutoState autoState = AutoState.OFF;
    private long autoTimer = 0;
    private int popCount = 0;

    // configurable timings (ms)
    private long autoInitialMoveDelayMs = 750;
    private long popUpMs = 150;
    private long popDownMs = 150;
    private long betweenRevolveMs = 200;
    private double autoRevolveDeg = 120.0;
    private int popRepeats = 3;

    private double lastRightBumperTarget = 60.0;
    private double lastLeftBumperTarget = 0.0;

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
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        popperServo = hardwareMap.get(Servo.class, "popperServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        shooter = new Shooter();
        shooter.init(hardwareMap, "motor1", "motor2",
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);

        // --- Mecanum drivetrain setup ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        double intakePower = 1.0;
        double incrementDeg = 120.0;
        double wheelRPM = 0.0;

        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;
        boolean lastLeftTriggerActive = false;

        telemetry.addLine("Initialized. Press START to begin.");
        telemetry.update();

        servoController.zeroNow();
        servoController.setKP(0.007);
        servoController.setKD(0.005);
        gateServo.setPosition(0.48);

        waitForStart();

        while (opModeIsActive()) {
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;

            // --- Mecanum drive control ---
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x * 1.1; // strafe correction
            double rx = gamepad1.right_stick_x; // rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            // --- Manual servo rotation ---
            if (autoState == AutoState.OFF) {
                if (dpadLeft && !lastDpadLeft) requestGateBeforeMove(-incrementDeg, false, 0);
                if (dpadRight && !lastDpadRight) requestGateBeforeMove(incrementDeg, false, 0);
                if (rightBumper && !lastRightBumper) requestGateBeforeMove(0, true, 60);
                if (leftBumper && !lastLeftBumper) requestGateBeforeMove(0, true, 0);
            }

            if (rightBumper && !lastRightBumper) lastRightBumperTarget = 60.0;
            if (leftBumper && !lastLeftBumper) lastLeftBumperTarget = 0.0;

            // --- Intake control ---
            if (gamepad1.y) intakeMotor.setPower(intakePower);
            else if (gamepad1.b) intakeMotor.setPower(-intakePower);
            else if (gamepad1.x) intakeMotor.setPower(0);

            // --- Popper manual control ---
            if (rightTrigger > 0.5 && autoState == AutoState.OFF)
                popperServo.setPosition(0.45);
            else if (autoState == AutoState.OFF)
                popperServo.setPosition(0.14);

            // --- Shooter RPM control (D-pad up/down) ---
            if (dpadUp && !lastDpadUp)
                wheelRPM = Range.clip(wheelRPM + 250, 0.0, 4900);
            if (dpadDown && !lastDpadDown)
                wheelRPM = Range.clip(wheelRPM - 250, 0.0, 4900);

            shooter.setTargetRPM(wheelRPM);
            shooter.update();

            // --- Auto-shoot trigger (left trigger) ---
            boolean leftTriggerActive = (leftTrigger > 0.7);
            if (leftTriggerActive && !lastLeftTriggerActive && autoState == AutoState.OFF) {
                autoState = AutoState.MOVE_TO_START;
                popCount = 0;
                gateServo.setPosition(0.38);
                servoController.moveServosToPosition(lastRightBumperTarget);
                autoTimer = System.currentTimeMillis() + autoInitialMoveDelayMs;
            }
            lastLeftTriggerActive = leftTriggerActive;

            // --- FSM updates ---
            updateGateFSM(servoController);
            updateAutoFSM(servoController);

            // --- Telemetry ---
            telemetry.clearAll();
            telemetry.addLine("=== System Status ===");
            telemetry.addData("Gate State", gateState);
            telemetry.addData("Servo Angle (deg)", "%.2f", servoController.getContinuousAngleDeg());
            telemetry.addData("Shooter Target RPM", "%.0f", wheelRPM);
            telemetry.addData("Shooter Current RPM", "%.0f", shooter.getMasterRPM());
            telemetry.addData("Gate Pos", "%.2f", gateServo.getPosition());
            telemetry.addData("Auto State", autoState);
            telemetry.addData("Pop Count", popCount);
            telemetry.addData("Drive FL/FR/BL/BR", "%.2f / %.2f / %.2f / %.2f", flPower, frPower, blPower, brPower);
            telemetry.update();

            // --- Memory update ---
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            lastRightBumper = rightBumper;
            lastLeftBumper = leftBumper;

            sleep(20);
        }

        servoController.stopAll();
        intakeMotor.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // ---------------- Gate FSM ----------------
    private void requestGateBeforeMove(double moveDeg, boolean absolute, double absTarget) {
        gateRequested = true;
        moveAmountDeg = moveDeg;
        useAbsolute = absolute;
        absoluteTarget = absTarget;
        if (absolute) {
            if (absTarget == 60.0) lastRightBumperTarget = absTarget;
            if (absTarget == 0.0) lastLeftBumperTarget = absTarget;
        }
    }

    private void updateGateFSM(ServoController servoController) {
        double gateDownPos = 0.38;
        double gateUpPos = 0.48;
        double delaySec = 0.12;
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
                        if (useAbsolute)
                            servoController.moveServosToPosition(absoluteTarget);
                        else
                            servoController.moveServosByRotation(moveAmountDeg);
                        servoMovePending = false;
                        gateTimer = now + 400;
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

        servoController.update();
    }

    // ---------------- Auto-shoot FSM ----------------
    private void updateAutoFSM(ServoController servoController) {
        long now = System.currentTimeMillis();

        switch (autoState) {
            case OFF:
                break;

            case MOVE_TO_START:
                if (now >= autoTimer) {
                    autoState = AutoState.POP_UP;
                    popCount = 0;
                    autoTimer = now + popUpMs;
                    popperServo.setPosition(0.45);
                }
                break;

            case POP_UP:
                if (now >= autoTimer) {
                    autoState = AutoState.POP_DOWN;
                    popperServo.setPosition(0.14);
                    autoTimer = now + popDownMs;
                }
                break;

            case POP_DOWN:
                if (now >= autoTimer) {
                    popCount++;
                    if (popCount < popRepeats) {
                        autoState = AutoState.REVOLVE;
                        autoTimer = now + betweenRevolveMs;
                    } else {
                        autoState = AutoState.RETURN_TO_START;
                        servoController.moveServosToPosition(lastLeftBumperTarget);
                        autoTimer = now + autoInitialMoveDelayMs;
                    }
                }
                break;

            case REVOLVE:
                if (now >= autoTimer) {
                    servoController.moveServosByRotation(autoRevolveDeg);
                    autoState = AutoState.WAIT_TO_SETTLE;
                    autoTimer = now + autoInitialMoveDelayMs;
                }
                break;

            case WAIT_TO_SETTLE:
                if (now >= autoTimer) {
                    autoState = AutoState.POP_UP;
                    popperServo.setPosition(0.45);
                    autoTimer = now + popUpMs;
                }
                break;

            case RETURN_TO_START:
                if (now >= autoTimer) {
                    autoState = AutoState.DONE;
                    servoController.moveServosToPosition(lastLeftBumperTarget);
                }
                break;

            case DONE:
                autoState = AutoState.OFF;
                popCount = 0;
                gateServo.setPosition(0.48);
                break;
        }

        servoController.update();
    }
}