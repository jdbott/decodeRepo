package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
public class ScrimTele extends LinearOpMode {

    // --- Subsystems / hardware ---
    private Servo popperServo;
    private Servo gateServo;
    private Shooter shooter;
    private Turret turret = new Turret();
    private ColorV3 colorSensor;

    // Drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // --- Shooter/turret constants ---
    public static double ff = 0.003;
    public static double kP = 0.008;

    private static final double INTAKE_POWER = 1.0;
    private static final double FLYWHEEL_RPM = 3300;

    private double fixedTurretAngle = 0;

    // --- Simple revolver logic (single source of truth) ---
    private double currentRevolverDeg = 0.0;
    private static final double STEP_120 = 120.0;
    private static final double OFFSET_60 = 60.0;
    private boolean shootOffsetActive = false;

    // --- Buttons edge memory ---
    private boolean lastDpadLeft = false, lastDpadRight = false;
    private boolean lastOptions = false, lastX = false;
    private boolean lastAButton = false, lastYButton = false;

    // --- Toggles and flags ---
    private boolean flywheelActive = false;
    private boolean intakeActive = false;
    private boolean gateLifted = false;

    // --- Gate FSM (kept intact for other actions; NOT used by revolver here) ---
    private enum GateState { IDLE, WAITING, RAISING }
    private GateState gateState = GateState.IDLE;
    private long gateTimer = 0;
    private boolean gateRequested = false;
    private boolean servoMovePending = false;
    private boolean useAbsolute = false;
    private double moveAmountDeg = 0;
    private double absoluteTarget = 0.0;

    // --- Auto FSM (left intact; independent of D-pad revolver logic) ---
    private enum AutoState { OFF, MOVE_TO_START, WAIT_TO_SETTLE, POP_UP, POP_DOWN, REVOLVE, DONE, RETURN_TO_START }
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

    // --- Color automation (kept) ---
    private boolean colorActive = true;
    private long colorRearmTime = 0;
    private int colorCount = 0;
    private static final int MAX_COLOR_DETECTIONS = 3;
    private static final long COLOR_REARM_DELAY_MS = 400;

    private GoBildaPinpointDriver pinpoint;

    // --- Helpers ---
    private void setRevolverTarget(ServoController servoController, double targetDeg) {
        currentRevolverDeg = targetDeg;
        servoController.moveServosToPosition(currentRevolverDeg);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware map
        ServoController servoController = new ServoController(hardwareMap);

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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        colorSensor = new ColorV3(hardwareMap);

        servoController.zeroNow();
        servoController.setKP(0.003);
        gateServo.setPosition(0.38);

        // Start state
        currentRevolverDeg = 0.0;
        setRevolverTarget(servoController, currentRevolverDeg);

        telemetry.addLine("Initialized. Press START to begin.");
        telemetry.update();
        waitForStart();

        double wheelRPM = 0.0;
        boolean lastLeftTriggerActive = false;

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // --- Field-centric drive ---
            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x * 1.1;
            double rx = gamepad2.right_stick_x;
            double trigger = Range.clip(1 - gamepad2.left_trigger, 0.2, 1);

            double botHeading = 1;
            double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotatedY = y * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            pinpoint.update();

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            frontLeft.setPower((y + x + rx) / denom * trigger);
            frontRight.setPower((y - x - rx) / denom * trigger);
            backLeft.setPower((y - x + rx) / denom * trigger);
            backRight.setPower((y + x - rx) / denom * trigger);

            if (gamepad2.share) pinpoint.resetPosAndIMU();

            boolean g2Left = gamepad2.dpad_left;
            boolean g2Right = gamepad2.dpad_right;
            boolean g2Down = gamepad2.dpad_down;

            if (g2Left) {
                currentRevolverDeg -= 5.0;
                servoController.moveServosToPosition(currentRevolverDeg);
            }
            if (g2Right) {
                currentRevolverDeg += 5.0;
                servoController.moveServosToPosition(currentRevolverDeg);
            }
            if (g2Down) {
                currentRevolverDeg = 0.0;
                servoController.zeroNow();
            }

            // =========================
            // SIMPLE REVOLVER BEHAVIOR
            // =========================
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            if (dpadLeft && !lastDpadLeft) {
                currentRevolverDeg -= STEP_120;        // current = current - 120
                requestGateBeforeMove(0, /*absolute*/ true, currentRevolverDeg);
            }
            if (dpadRight && !lastDpadRight) {
                currentRevolverDeg += STEP_120;        // current = current + 120
                requestGateBeforeMove(0, /*absolute*/ true, currentRevolverDeg);
            }

            // Options: enter shoot offset and spin up flywheel, stop intake
            boolean optionsPressed = gamepad1.options;
            if (optionsPressed && !lastOptions) {
                if (!shootOffsetActive) {
                    currentRevolverDeg += OFFSET_60;   // +60 once
                    shootOffsetActive = true;
                }
                // keep previous actions
                intakeActive = false;
                intakeMotor.setPower(0.0);
                gateServo.setPosition(0.38);
                flywheelActive = true;
                wheelRPM = FLYWHEEL_RPM;
                shooter.setTargetRPM(wheelRPM);
            }
            lastOptions = optionsPressed;

            // X: exit shoot offset and start intake, stop flywheel
            boolean xPressed = gamepad1.x;
            if (xPressed && !lastX) {
                if (shootOffsetActive) {
                    currentRevolverDeg -= OFFSET_60;   // -60 once
                    shootOffsetActive = false;
                }
                // keep previous actions
                intakeActive = true;
                intakeMotor.setPower(1.0);
                gateServo.setPosition(0.48);
                flywheelActive = false;
                wheelRPM = 0.0;
                shooter.setTargetRPM(wheelRPM);
            }
            lastX = xPressed;

            // Exactly one revolver command per loop
            servoController.moveServosToPosition(currentRevolverDeg);

            // --- Popper ---
            double rightTrigger = gamepad1.right_trigger;
            if (rightTrigger > 0.5 && autoState == AutoState.OFF)
                popperServo.setPosition(0.45);
            else if (autoState == AutoState.OFF)
                popperServo.setPosition(0.14);

            // --- Intake toggle (Y) ---
            boolean yButton = gamepad1.y;
            if (yButton && !lastYButton) {
                intakeActive = !intakeActive;
                if (intakeActive) {
                    intakeMotor.setPower(1.0);
                    gateServo.setPosition(0.48);
                    gateLifted = true;
                } else {
                    intakeMotor.setPower(0.0);
                    gateServo.setPosition(0.38);
                    gateLifted = false;
                }
            }
            lastYButton = yButton;

            // --- Quick outtake (B) ---
            if (gamepad1.b) {
                gateServo.setPosition(0.48);
                intakeMotor.setPower(-INTAKE_POWER);
                intakeActive = false;
            }

            // --- Flywheel toggle (A) ---
            boolean aButton = gamepad1.a;
            if (aButton && !lastAButton) {
                flywheelActive = !flywheelActive;
                wheelRPM = flywheelActive ? FLYWHEEL_RPM : 0.0;
                shooter.setTargetRPM(wheelRPM);
            }
            lastAButton = aButton;

            // --- RPM nudge ---
            if (gamepad1.dpad_up)   wheelRPM = Range.clip(wheelRPM + 250, 0.0, 4900);
            if (gamepad1.dpad_down) wheelRPM = Range.clip(wheelRPM - 250, 0.0, 4900);
            shooter.setTargetRPM(wheelRPM);
            shooter.update();

            // --- Auto shoot trigger (kept, independent) ---
            double leftTrigger = gamepad1.left_trigger;
            boolean leftTriggerActive = (leftTrigger > 0.7);
            if (leftTriggerActive && !lastLeftTriggerActive && autoState == AutoState.OFF) {
                autoState = AutoState.MOVE_TO_START;
                popCount = 0;
                gateServo.setPosition(0.38);
                // move to a start point for auto; use current as baseline
                setRevolverTarget(servoController, currentRevolverDeg);
                autoTimer = now + autoInitialMoveDelayMs;
            }
            lastLeftTriggerActive = leftTriggerActive;

            // --- Color automation (kept minimal) ---
            boolean intakeRunning = intakeActive; // simple coupling
            if (intakeRunning && colorActive && now >= colorRearmTime && colorCount < MAX_COLOR_DETECTIONS) {
                String color = colorSensor.proximityAndColor();
                if (color.equals("Green") || color.equals("Purple")) {
                    currentRevolverDeg -= 120.0; // advance bin
                    servoController.moveServosToPosition(currentRevolverDeg);
                    colorCount++;
                    colorActive = false;
                    colorRearmTime = now + COLOR_REARM_DELAY_MS;
                }
            } else if (!colorActive && now >= colorRearmTime) {
                colorActive = true;
            }
            if (gamepad1.share) { // reset color counters
                colorCount = 0;
                colorActive = true;
            }

            // --- Gate and Auto FSM updates (kept) ---
            updateGateFSM(servoController);
            updateAutoFSM(servoController);

            // --- Turret loop (kept) ---
            turret.setAngle(fixedTurretAngle);
            turret.update();
            turret.setKF(ff);
            turret.setKP(kP);

            // --- Telemetry ---
            telemetry.clearAll();
            telemetry.addLine("=== Simple Revolver Control ===");
            telemetry.addData("Target (deg)", "%.1f", currentRevolverDeg);
            telemetry.addData("Shoot Offset", shootOffsetActive);
            telemetry.addLine("=== Shooter/Intake ===");
            telemetry.addData("Flywheel Active", flywheelActive);
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Shooter Target RPM", "%.0f", wheelRPM);
            telemetry.addLine("=== Gate/Auto ===");
            telemetry.addData("Gate State", gateState);
            telemetry.addData("Auto State", autoState);
            telemetry.addLine("=== Color ===");
            telemetry.addData("Detections", "%d/%d", colorCount, MAX_COLOR_DETECTIONS);
            telemetry.update();

            // Edge memory
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;

            sleep(20);
        }

        servoController.stopAll();
    }

    // ---------------- Gate FSM (unchanged; not used by revolver logic) ----------------
    private void requestGateBeforeMove(double moveDeg, boolean absolute, double absTarget) {
        gateRequested = true;
        moveAmountDeg = moveDeg;
        useAbsolute = absolute;
        absoluteTarget = absTarget;
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
                    setRevolverTarget(servoController, absoluteTarget);
                    servoMovePending = false;
                    gateTimer = now + 400;
                    gateState = GateState.RAISING;
                }
                break;

            case RAISING:
                if (now >= gateTimer) {
                    if (!intakeActive) {
                        gateServo.setPosition(gateUpPos);
                    }
                    gateState = GateState.IDLE;
                }
                break;
        }

        servoController.update();
    }

    // ---------------- Auto FSM (kept) ----------------
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
                        setRevolverTarget(servoController, currentRevolverDeg);
                        autoTimer = now + autoInitialMoveDelayMs;
                    }
                }
                break;

            case REVOLVE:
                if (now >= autoTimer) {
                    setRevolverTarget(servoController, currentRevolverDeg + autoRevolveDeg);
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
                    setRevolverTarget(servoController, currentRevolverDeg);
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