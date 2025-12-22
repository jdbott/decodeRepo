package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import com.pedropathing.paths.Path;

@TeleOp(name = "A RED TELEOP")
public class ScrimTeleRed extends LinearOpMode {

    // --- Subsystems / hardware ---
    private Follower follower;
    private static double TARGET_X = 71;
    private static double TARGET_Y = 69;
    private Servo popperServo;
    private Servo gateServo;
    private Shooter shooter;
    private Turret turret = new Turret();
    private ColorV3 colorSensor;

    private Limelight3A limelight3A;

    // Drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // --- Shooter/turret constants ---
    public static double ff = 0.003;
    public static double kP = 0.008;

    private static final double FLYWHEEL_RPM = 3300;


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
    private boolean colorActive = false;
    private long colorRearmTime = 0;
    private int colorCount = 4;
    private static final int MAX_COLOR_DETECTIONS = 3;
    private static final long COLOR_REARM_DELAY_MS = 400;

    private enum ParkState { OFF, RUNNING }
    private ParkState parkState = ParkState.OFF;

    private boolean lastStickPress = false;
    private Path parkPath;

    private void setRevolverTarget(ServoController servoController, double targetDeg) {
        currentRevolverDeg = targetDeg;
        servoController.moveServosToPosition(currentRevolverDeg);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(35, 1, Math.toRadians(0)));
        follower.updatePose();
        follower.setMaxPower(1);

        // Hardware map
        ServoController servoController = new ServoController(hardwareMap);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

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
        turret.setLimits(-205, 90);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
        imu.resetYaw();

        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.setPollRateHz(50);

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
        limelight3A.start();
        turret.setAngle(0);

        double wheelRPM = 0.0;
        boolean lastLeftTriggerActive = false;

        resetRuntime();
        boolean bPressed = false;

        while (opModeIsActive()) {
            follower.update();

            // Robot pose
            double botX = follower.getPose().getX();
            double botY = follower.getPose().getY();
            double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());  // standard math frame

            // Vector to target
            double dx = TARGET_X - botX;
            double dy = TARGET_Y - botY;

            // Absolute field angle toward target (0°=+X, CCW positive)
            double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));  // correct for math coordinate system

            // Turret angle = field angle - robot heading
            double turretAngleNeeded = normalize180(angleToTarget - robotHeadingDeg);

            // Drive turret
            if (wheelRPM != 0 && !gamepad2.a) {
                turret.setAngle(-turretAngleNeeded);
            } else {
                turret.setAngle(0);
            }
            turret.update();

            if (flywheelActive) {
                // Use EXACTLY the same angle the turret control loop is receiving
                double cmd = (wheelRPM != 0 && !gamepad2.a)
                        ? -turretAngleNeeded
                        : 0;

                double limited = Range.clip(cmd, -205, 90);
                boolean hardStopping = (cmd != limited);

                if (hardStopping) {
                    gamepad1.rumble(1.0, 1.0, 50);
                    gamepad2.rumble(1.0, 1.0, 50);
                }
            }

            long now = System.currentTimeMillis();

            double rotatedX = 0;
            double rotatedY = 0;
            double x = 0;
            double y = 0;
            double rx = 0;

            double trigger = Range.clip(1 - gamepad2.right_trigger, 0.2, 1);

            if (!(gamepad2.left_trigger > 0.5)) {
                y = -gamepad2.left_stick_y;
                x = gamepad2.left_stick_x;
                rx = gamepad2.right_stick_x;

                double botHeading = Math.toRadians(robotHeadingDeg);
                rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            } else {
                y = -gamepad2.left_stick_y;
                x = gamepad2.left_stick_x;
                rx = gamepad2.right_stick_x;
                trigger = Range.clip(1 - gamepad2.right_trigger, 0.2, 1);

                rotatedX = x;
                rotatedY = y;
            }

            if (gamepad2.b) {
                bPressed = true;
                follower.setPose(new Pose(46.9, 55.1, Math.toRadians(137.7-90)));
                gamepad2.rumble(500);
            }

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            frontLeft.setPower((rotatedY + rotatedX + rx) / denom * trigger);
            frontRight.setPower((rotatedY - rotatedX - rx) / denom * trigger);
            backLeft.setPower((rotatedY - rotatedX + rx) / denom * trigger);
            backRight.setPower((rotatedY + rotatedX - rx) / denom * trigger);

//            boolean stickPress = gamepad1.left_stick_button;
//
//            // Toggle logic
//            if (stickPress && !lastStickPress) {
//                if (parkState == ParkState.OFF) {
//                    startParkFSM();
//                } else {
//                    stopParkFSM();
//                }
//            }
//            lastStickPress = stickPress;

            // FSM update
            if (parkState == ParkState.RUNNING) {
                // Disable teleop drive while auto path is running
                if (!follower.isBusy()) {
                    stopParkFSM();
                } else {
                    // Skip user drive so auto can execute
                    // But DO NOT block the loop — follower.update() still runs above
                    continue;
                }
            }

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
            boolean optionsPressed = gamepad1.a;
            if (optionsPressed && !lastOptions) {
                if (!shootOffsetActive) {
                    currentRevolverDeg = -180;   // +60 once
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
                    currentRevolverDeg = 0;
                    shootOffsetActive = false;
                }
                // keep previous actions
                intakeActive = true;
                intakeMotor.setPower(-1.0);
                gateServo.setPosition(0.48);
                flywheelActive = false;
                wheelRPM = 0.0;
                shooter.setTargetRPM(wheelRPM);
                colorCount = 0;
                colorActive = true;
            }
            lastX = xPressed;

            // Exactly one revolver command per loop
            servoController.moveServosToPosition(currentRevolverDeg);

            // --- Popper ---
            double rightTrigger = gamepad1.right_trigger;
            if (rightTrigger > 0.5 && autoState == AutoState.OFF)
                popperServo.setPosition(0.45);
            else if (autoState == AutoState.OFF)
                popperServo.setPosition(0.15);

            // --- Intake toggle (Y) ---
            boolean yButton = gamepad1.y;
            if (yButton && !lastYButton) {
                intakeActive = !intakeActive;
                if (intakeActive) {
                    intakeMotor.setPower(-1.0);
                    gateServo.setPosition(0.48);
                    gateLifted = true;
                } else {
                    intakeMotor.setPower(0.0);
                    gateServo.setPosition(0.38);
                    gateLifted = false;
                }
            }
            lastYButton = yButton;

            // --- RPM nudge ---
            if (gamepad1.dpad_up) {
                wheelRPM = 3850;
                TARGET_X = 66;
            }
            if (gamepad1.dpad_down){
                wheelRPM = FLYWHEEL_RPM;
                TARGET_X = 71;
            }
            shooter.setTargetRPM(wheelRPM);
            shooter.update();

            // --- Auto shoot trigger (kept, independent) ---
            double leftTrigger = 0;
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
                    if (colorCount < 3) {
                        currentRevolverDeg -= 120.0; // advance bin
                        requestGateBeforeMove(0, true, currentRevolverDeg);
                    }
                    colorCount++;
                    colorActive = false;
                    colorRearmTime = now + COLOR_REARM_DELAY_MS;
                }
            } else if (!colorActive && now >= colorRearmTime) {
                colorActive = true;
            } else if (colorCount == 3) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                intakeMotor.setPower(0);
                gateServo.setPosition(0.38);
                colorCount++;
                currentRevolverDeg = 60;
                requestGateBeforeMove(0, true, currentRevolverDeg);
            }
            if (gamepad1.options) {
                colorCount = 0;
                colorActive = true;
            }

//            limelight3A.updateRobotOrientation(Math.toRadians(robotHeadingDeg));
//            LLResult result = limelight3A.getLatestResult();
//
//            telemetry.addData("LL Yaw", result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.DEGREES));
//            telemetry.addData("LL Distance x", result.getBotpose_MT2().getPosition().x);
//            telemetry.addData("LL Distance y", result.getBotpose_MT2().getPosition().y);
//            telemetry.addData("LL Distance z", result.getBotpose_MT2().getPosition().z);
//            telemetry.addData("Distance", result.getBotposeAvgDist());
//
//            if (result.getBotposeAvgDist() != 0.0) {
//                distance = result.getBotposeAvgDist()*1000;
//            } else {
//                distance = 0;
//            }
//
//
//            if (gamepad1.b) {
//                wheelRPM = -2.08318e-10 * Math.pow(x, 4)
//                        + 0.00000142366 * Math.pow(x, 3)
//                        - 0.00296486 * Math.pow(x, 2)
//                        + 1.93708 * x
//                        + 3520.10779;
//            }
//            double yaw = result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.DEGREES);
//
//            // if target is visible, use its yaw to correct
//            if (Math.abs(yaw) > 0.5) { // tolerance to ignore small noise
//                desiredFieldHeading = robotHeadingDeg + yaw;
//                lastEdgeAngle = desiredFieldHeading;
//            } else {
//                // yaw == 0 or target lost, hold last known direction
//                desiredFieldHeading = 0;
//            }
//
//            // convert back to turret-relative command
//            double targetTurretAngle = desiredFieldHeading;
//            targetTurretAngle = Range.clip(targetTurretAngle, lowerLimit, upperLimit);
//
//            turret.update();

            if (gamepad1.left_trigger > 0.5) {
                autoState = AutoState.MOVE_TO_START;
            }

            // --- Gate and Auto FSM updates (kept) ---
            updateGateFSM(servoController);
            updateAutoFSM(servoController);

            // --- Turret loop (kept) ---
            turret.update();
            turret.setKF(ff);
            turret.setKP(kP);

            // --- Telemetry ---
            telemetry.addLine("=== Simple Revolver Control ===");
            telemetry.addData("Target (deg)", "%.1f", currentRevolverDeg);
            telemetry.addData("Shoot Offset", shootOffsetActive);
            telemetry.addLine("=== Shooter/Intake ===");
            telemetry.addData("Flywheel Active", flywheelActive);
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Shooter Target RPM", "%.0f", wheelRPM);
            telemetry.addData("Current Shooter RPM", shooter.getMasterRPM());
            telemetry.addData("Curret Shooter Power", shooter.getLastPower());
            telemetry.addLine("=== Gate/Auto ===");
            telemetry.addData("Gate State", gateState);
            telemetry.addData("Auto State", autoState);
            telemetry.addLine("=== Color ===");
            telemetry.addData("Detections", "%d/%d", colorCount, MAX_COLOR_DETECTIONS);
            telemetry.addData("Raw Data", colorSensor.proximityAndColor());
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.update();

            // Edge memory
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;

            sleep(20);
        }

        servoController.stopAll();
    }

    private void startParkFSM() {
        Pose p = follower.getPose();

        // build the path
        parkPath = new Path(new BezierLine(
                new Pose(p.getX(), p.getY(), p.getHeading()),
                new Pose(-36, -12, 0)
        ));
        parkPath.setConstantHeadingInterpolation(Math.toRadians(90));

        follower.breakFollowing();
        //follower.followPath(parkPath, true);

        parkState = ParkState.RUNNING;
    }

    private void stopParkFSM() {
        follower.breakFollowing();
        follower.startTeleOpDrive(); // back to normal user control
        parkState = ParkState.OFF;
    }

    private double normalize180(double a) {
        a = ((a + 180) % 360 + 360) % 360 - 180;
        return a;
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
                    gateServo.setPosition(gateUpPos);
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
                    popperServo.setPosition(0.15);
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