package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "A Srimmage teleop")
public class IntakeCR extends LinearOpMode {

    private Servo popperServo;
    private Servo gateServo;
    private Shooter shooter;
    private Turret turret = new Turret();
    private ColorV3 colorSensor;

    // --- Drivetrain motors ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private double targetHeading = 0;
    private double turretOffset = 0;
    private double fixedTurretAngle = 0;

    public static double ff = 0.003;
    public static double kP = 0.008;

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

    private long autoInitialMoveDelayMs = 650;
    private long postRevolveDelayMs = 600;
    private long popUpMs = 250;
    private long popDownMs = 250;
    private long betweenRevolveMs = 0;
    private double autoRevolveDeg = 120.0;
    private int popRepeats = 3;

    private double lastRightBumperTarget = 180;
    private double lastLeftBumperTarget = 0.0;

    // --- Color automation variables ---
    private boolean colorActive = true;
    private long colorRearmTime = 0;
    private int colorCount = 0;
    private static final int MAX_COLOR_DETECTIONS = 3;
    private static final long COLOR_REARM_DELAY_MS = 400; // 0.2 sec

    // --- New Toggles ---
    private boolean flywheelActive = false;
    private boolean lastAButton = false;
    private boolean intakeActive = false;
    private boolean lastYButton = false;

    private static final double INTAKE_POWER = 1.0;
    private static final double FLYWHEEL_RPM = 3100;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware setup ---
        ServoController servoController = new ServoController(
                hardwareMap,
                new String[]{"CRServo1", "CRServo2"},
                new double[]{+1.0, -1.0},
                "CRServo1A"
        );

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        popperServo = hardwareMap.get(Servo.class, "popperServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        shooter = new Shooter();
        shooter.init(hardwareMap, "motor1", "motor2",
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE);

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

        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.setKP(kP);
        turret.setKF(ff);
        turret.setLimits(-180, 180);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
        imu.resetYaw();

        colorSensor = new ColorV3(hardwareMap);

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        targetHeading = orientation.getYaw(AngleUnit.DEGREES);

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

        // Fixed turret reference angle
        fixedTurretAngle = 0;

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // --- Field-centric drive ---
            YawPitchRollAngles orientationNow = imu.getRobotYawPitchRollAngles();
            double botHeading = orientationNow.getYaw(AngleUnit.RADIANS);

            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x * 1.1;
            double rx = gamepad2.right_stick_x;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            frontLeft.setPower((rotY + rotX + rx) / denom);
            frontRight.setPower((rotY - rotX - rx) / denom);
            backLeft.setPower((rotY - rotX + rx) / denom);
            backRight.setPower((rotY + rotX - rx) / denom);

            // --- IMU Reset ---
            if (gamepad2.share) {
                imu.resetYaw();
            }

            if (gamepad1.x) {
                intakeMotor.setPower(1);
                wheelRPM = 0;
                servoController.moveServosToPosition(0);
            }

            if (gamepad1.options) {
                intakeMotor.setPower(0);
                wheelRPM = FLYWHEEL_RPM;
                requestGateBeforeMove(0, true, 60);
            }

            // --- Manual servo control (same as before) ---
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;
            boolean sharePressed = gamepad1.share;

            if (autoState == AutoState.OFF) {
                if (dpadLeft && !lastDpadLeft)
                    requestGateBeforeMove(-incrementDeg, false, 0);
                if (dpadRight && !lastDpadRight)
                    requestGateBeforeMove(incrementDeg, false, 0);
                if (rightBumper && !lastRightBumper)
                    requestGateBeforeMove(0, true, 60);
                if (leftBumper && !lastLeftBumper)
                    requestGateBeforeMove(0, true, 0);
            }

            if (rightBumper && !lastRightBumper) lastRightBumperTarget = 60.0;
            if (leftBumper && !lastLeftBumper) lastLeftBumperTarget = 0.0;

            // --- Intake toggle logic ---
            boolean yButton = gamepad1.y;
            if (yButton && !lastYButton) {
                intakeActive = !intakeActive;
                intakeMotor.setPower(intakeActive ? INTAKE_POWER : 0);
            }
            lastYButton = yButton;

            if (gamepad1.b) {
                intakeMotor.setPower(-INTAKE_POWER);
                intakeActive = false;
            }

            // --- Popper manual ---
            if (rightTrigger > 0.5 && autoState == AutoState.OFF)
                popperServo.setPosition(0.45);
            else if (autoState == AutoState.OFF)
                popperServo.setPosition(0.14);

            // --- Flywheel toggle (A) ---
            boolean aButton = gamepad1.a;
            if (aButton && !lastAButton) {
                flywheelActive = !flywheelActive;
                wheelRPM = flywheelActive ? FLYWHEEL_RPM : 0;
                shooter.setTargetRPM(wheelRPM);
            }
            lastAButton = aButton;

            // --- Shooter RPM adjustments ---
            if (dpadUp && !lastDpadUp) wheelRPM = Range.clip(wheelRPM + 250, 0.0, 4900);
            if (dpadDown && !lastDpadDown) wheelRPM = Range.clip(wheelRPM - 250, 0.0, 4900);
            shooter.setTargetRPM(wheelRPM);
            shooter.update();

            // --- Auto shoot trigger ---
            boolean leftTriggerActive = (leftTrigger > 0.7);
            if (leftTriggerActive && !lastLeftTriggerActive && autoState == AutoState.OFF) {
                autoState = AutoState.MOVE_TO_START;
                popCount = 0;
                gateServo.setPosition(0.38);
                servoController.moveServosToPosition(lastRightBumperTarget);
                autoTimer = now + autoInitialMoveDelayMs;
            }
            lastLeftTriggerActive = leftTriggerActive;

            // --- Color automation logic (unchanged) ---
            boolean intakeRunning = false;
            if (intakeRunning && colorActive && now >= colorRearmTime && colorCount < MAX_COLOR_DETECTIONS) {
                String color = colorSensor.proximityAndColor();
                if (color.equals("Green") || color.equals("Purple")) {
                    requestGateBeforeMove(-120.0, false, 0);
                    colorCount++;
                    colorActive = false;
                    colorRearmTime = now + COLOR_REARM_DELAY_MS;
                }
            } else if (!colorActive && now >= colorRearmTime) {
                colorActive = true;
            }

            if (sharePressed) {
                colorCount = 0;
                colorActive = true;
            }

            // --- FSM updates ---
            updateGateFSM(servoController);
            updateAutoFSM(servoController);

            // --- Turret hold fixed ---
            turret.setAngle(fixedTurretAngle);
            turret.update();
            turret.setKF(ff);
            turret.setKP(kP);

            // --- Telemetry ---
            telemetry.clearAll();
            telemetry.addLine("=== System Status ===");
            telemetry.addData("Gate State", gateState);
            telemetry.addData("Servo Angle (deg)", "%.2f", servoController.getContinuousAngleDeg());
            telemetry.addData("Shooter Target RPM", "%.0f", wheelRPM);
            telemetry.addData("Flywheel Active", flywheelActive);
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Color Detected", colorSensor.proximityAndColor());
            telemetry.addData("Color Detections", "%d / %d", colorCount, MAX_COLOR_DETECTIONS);
            telemetry.addData("Color Active", colorActive);
            telemetry.addData("IMU Yaw", "%.1f°", orientationNow.getYaw(AngleUnit.DEGREES));
            telemetry.update();

            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            lastRightBumper = rightBumper;
            lastLeftBumper = leftBumper;

            sleep(20);
        }

        servoController.stopAll();
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
                if (now >= gateTimer && servoMovePending) {
                    if (useAbsolute)
                        servoController.moveServosToPosition(absoluteTarget);
                    else
                        servoController.moveServosByRotation(moveAmountDeg);
                    servoMovePending = false;
                    gateTimer = now + 400;
                    gateState = GateState.RAISING;
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
                    autoTimer = now + postRevolveDelayMs;
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