package org.firstinspires.ftc.teamcode.teles;

import org.firstinspires.ftc.teamcode.RobotConfig;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AllianceMirror;
import org.firstinspires.ftc.teamcode.AllianceStore;
import org.firstinspires.ftc.teamcode.AutoStartStore;
import org.firstinspires.ftc.teamcode.hardwareClasses.Feeder;
import org.firstinspires.ftc.teamcode.hardwareClasses.Flywheel;
import org.firstinspires.ftc.teamcode.hardwareClasses.Hood;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "A V3 Tele")
public class V3Tele extends LinearOpMode {

    private Hood hood;
    private Feeder feeder;

    private Follower follower;
    private Turret turret;

    private Intake intake;
    private Flywheel flywheel;

    private boolean isRedAlliance = false;

    // Starts in auto shot tracking mode
    private enum ShotMode {
        AUTO_SWITCH,
        CLOSE,
        FAR
    }

    // Start in automatic switching mode
    private ShotMode shotMode = ShotMode.AUTO_SWITCH;

    // Automatic FAR override based on pose + distance, only used in AUTO_SWITCH mode
    private boolean autoFarOverride = false;

    // Toggle this to enable rudimentary shooting-on-the-move aim compensation
    private static final boolean ENABLE_SHOT_ON_MOVE_COMP = true;

    // Fixed override shot settings
    private static final double FIXED_POWER_SHOT_VELOCITY_RAD = 440;
    private static final double FIXED_POWER_SHOT_HOOD_DEG = 53.5;

    // Light filtering for velocity estimate
    private static final double VELOCITY_FILTER_ALPHA = 0.25;

    // Very light smoothing for predicted shot distance
    private static final double PREDICTED_DISTANCE_ALPHA = 0.45;
    private static double TIME_TUNER = 0.77;

    // Intake toggle
    private boolean lastX = false;
    private boolean intakeToggleOn = false;

    // Feed sequence trigger
    private boolean lastB = false;

    // Debounce for hood adjustment
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Debounce for shot mode toggle
    private boolean lastRightBumper1 = false;
    private boolean lastRightBumper2 = false;

    // Timed shoot/feed sequence
    private final ElapsedTime sequenceTimer = new ElapsedTime();

    // Pose-based velocity estimation
    private final ElapsedTime poseVelocityTimer = new ElapsedTime();
    private boolean velocityInitialized = false;
    private double lastPoseX = 0.0;
    private double lastPoseY = 0.0;
    private double fieldVxInPerSec = 0.0;
    private double fieldVyInPerSec = 0.0;

    // Predicted distance state for hood/flywheel shot-on-move compensation
    private double predictedShotDistance = 0.0;
    private double filteredPredictedShotDistance = 0.0;
    private boolean predictedDistanceInitialized = false;
    private double radialVelocityToGoal = 0.0;

    // Desired turret angular velocity estimation
    private final ElapsedTime turretTargetVelTimer = new ElapsedTime();
    private boolean turretTargetVelInitialized = false;
    private double lastDesiredTurretDeg = 0.0;
    private double desiredTurretVelDegPerSec = 0.0;
    private static final double TURRET_TARGET_VEL_FILTER_ALPHA = 0.8;

    private boolean oneDriver = false;

    // Debounce for turret offset adjustment
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Debounce for zeroing coarse adjust buttons
    private boolean lastSquareZero = false;
    private boolean lastCircleZero = false;

    private boolean lastSquareFlywheel = false;
    private boolean lastCircleFlywheel = false;

    // Debounce for auto park toggle
    private boolean lastTrianglePark = false;

    // Manual turret trim in degrees
    private double turretAngleOffsetDeg = 3;

    // Turret zeroing mode
    private boolean turretZeroingMode = false;
    private double turretZeroingTargetDeg = 0.0;
    private boolean lastTurretZeroingToggle = false;
    private static final double TURRET_ZERO_STEP_DEG = 4.0;
    private static final double TURRET_ZERO_BIG_STEP_DEG = 10.0;

    // Auto-park state
    private boolean autoParkActive = false;
    private boolean autoParkSettling = false;
    private Path autoParkPath;
    private final ElapsedTime autoParkSettleTimer = new ElapsedTime();
    private static final double AUTO_PARK_SETTLE_TIME_SEC = 0.5;
    private enum FeedState {
        IDLE,
        WAIT_BEFORE_INTAKE,
        RUN_INTAKE,
        DONE
    }

    private FeedState feedState = FeedState.IDLE;

    // Hood angle state
    private double hoodAngleDeg = 50.0;

    // Flywheel target velocity state
    private double targetVelocityRad = 275;

    // Goal position in field coordinates
    private static final double BLUE_TARGET_X = 5;
    private static final double TARGET_Y = 139;

    // Turret center offset from robot center (inches)
    // Behind robot center by 1.5 in along robot heading direction
    private static final double TURRET_CENTER_OFFSET_IN = 1.5;

    // Turret limits
    private static final double TURRET_MIN_DEG = -175;
    private static final double TURRET_MAX_DEG = 175;

    private double baseTurretAngleOffsetDeg = 1;

    // This stays the same unless your turret's own mechanical frame needs changing
    private static final double TURRET_OFFSET_DEG = 180.0;

    // Blue-native teleop zero poses based on which auto was run
    private static final Pose CLOSE_AUTO_TELEOP_START_BLUE =
            new Pose(57.8, 111.2, Math.toRadians(-90));

    private static final Pose FAR_AUTO_TELEOP_START_BLUE =
            new Pose(41.1, 12.25, Math.toRadians(180));

    // Blue-native auto-park pose
    private static final Pose AUTO_PARK_POSE_BLUE =
            new Pose(105, 32.0, Math.toRadians(-90.0));

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);

        hood = new Hood(hardwareMap);
        feeder = new Feeder(hardwareMap);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new Flywheel(hardwareMap, battery);

        isRedAlliance = AllianceStore.isRed(hardwareMap.appContext);
        boolean isCloseAuto = AutoStartStore.isClose(hardwareMap.appContext);

        follower = Constants.createFollower(hardwareMap);

        Pose blueTeleopStartPose = isCloseAuto
                ? CLOSE_AUTO_TELEOP_START_BLUE
                : FAR_AUTO_TELEOP_START_BLUE;

        Pose startPose = AllianceMirror.mirrorPose(blueTeleopStartPose, isRedAlliance);

        follower.setStartingPose(startPose);
        follower.updatePose();
        follower.setMaxPower(1);
        follower.startTeleOpDrive();

        turret = new Turret(hardwareMap, RobotConfig.TURRET_MOTOR, DcMotorSimple.Direction.REVERSE);

        feeder.clutchOut();
        feeder.armBlock();
        intake.setPower(0.0);

        telemetry.addLine("Initialized");
        telemetry.addLine("Clutch out, arm blocked, intake off");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Auto Start Mode", isCloseAuto ? "CLOSE" : "FAR");
        telemetry.addData("Teleop Start Pose", startPose);
        telemetry.addData("Shot Mode", shotMode);
        telemetry.addData("Shot On Move Compensation", ENABLE_SHOT_ON_MOVE_COMP);
        telemetry.update();

        while (opModeInInit()) {
            turret.update();

            if (gamepad1.x) {
                oneDriver = true;
            } else if (gamepad1.b) {
                oneDriver = false;
            }

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Auto Start Mode", isCloseAuto ? "CLOSE" : "FAR");
            telemetry.addData("Teleop Start Pose", startPose);
            telemetry.addData("One Driver", oneDriver);
            telemetry.addData("Shot Mode", shotMode);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        intake.setPower(1);
        flywheel.setTargetVelocity(targetVelocityRad);

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();

            fieldVxInPerSec = follower.getVelocity().getXComponent();
            fieldVyInPerSec = follower.getVelocity().getYComponent();
            double robotAngularVelRadPerSec = follower.getAngularVelocity();

            updateAutoFarOverride(pose);

            handleAutoPark(pose);

            if (!autoParkActive) {
                double robotHeadingDeg = Math.toDegrees(pose.getHeading());
                drive(robotHeadingDeg);
            }

            handleTurretZeroingMode();

            if (!turretZeroingMode) {
                handleTurretOffsetAdjustment();
                trackGoalFromOdometry(pose, robotAngularVelRadPerSec);
            }

            turret.update();

            if (!turretZeroingMode) {
                handleShotModeToggle();
            }

            boolean useFarShot;
            switch (shotMode) {
                case FAR:
                    useFarShot = true;
                    break;

                case CLOSE:
                    useFarShot = false;
                    break;

                case AUTO_SWITCH:
                default:
                    useFarShot = autoFarOverride;
                    break;
            }

            if (useFarShot) {
                targetVelocityRad = FIXED_POWER_SHOT_VELOCITY_RAD;
                hoodAngleDeg = FIXED_POWER_SHOT_HOOD_DEG;
//    handleHoodAdjustment();
//    handleFlywheelAdjustment();
                hoodAngleDeg = hood.setAngle(hoodAngleDeg);
                baseTurretAngleOffsetDeg = 1;
            } else {
                double lookupDistance = predictedDistanceInitialized
                        ? filteredPredictedShotDistance
                        : Math.hypot(getTargetX() - pose.getX(), TARGET_Y - pose.getY());
                updateShotFromDistance(lookupDistance);
                baseTurretAngleOffsetDeg = 1;
            }

            if (!turretZeroingMode) {
                handleXToggle();
                handleBSequence();
            }

            updateFeedSequence();

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Shot Mode", shotMode);
            telemetry.addData("Auto FAR Override Active", autoFarOverride);
            telemetry.addData("Effective Shot",
                    (shotMode == ShotMode.FAR || (shotMode == ShotMode.AUTO_SWITCH && autoFarOverride))
                            ? "FAR" : "CLOSE");
            telemetry.addData("Turret Zeroing Mode", turretZeroingMode);
            telemetry.addData("Auto Park Active", autoParkActive);
            telemetry.addData("Auto Park Target", getAutoParkPose());
            telemetry.addData("Turret Offset (deg)", turretAngleOffsetDeg);
            telemetry.addData("Hood Angle (deg)", hoodAngleDeg);
            telemetry.addData("Flywheel Target (rad/s)", targetVelocityRad);
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.update();
        }

        flywheel.stop();
        intake.setPower(0.0);
    }

    private double getTargetX() {
        return AllianceMirror.mirrorX(BLUE_TARGET_X, isRedAlliance);
    }

    private Pose getAutoParkPose() {
        return AllianceMirror.mirrorPose(AUTO_PARK_POSE_BLUE, isRedAlliance);
    }

    private void updateAutoFarOverride(Pose pose) {
        double distanceToGoal = Math.hypot(getTargetX() - pose.getX(), TARGET_Y - pose.getY());
        autoFarOverride = distanceToGoal > 120.0 && pose.getY() < 48.0;
    }

    private boolean getZeroingTogglePressed() {
        return oneDriver ? gamepad1.dpad_up : gamepad2.dpad_up;
    }

    private boolean getAutoParkTogglePressed() {
        if (oneDriver) {
            return gamepad1.triangle;
        }
        return gamepad1.triangle || gamepad1.triangle;
    }

    private void handleFlywheelAdjustment() {
        // Example: operator uses square/circle on gamepad2
        boolean flywheelUpPressed = gamepad1.options;
        boolean flywheelDownPressed = gamepad1.share;

        if (flywheelUpPressed && !lastSquareFlywheel) {
            targetVelocityRad += 10.0;
        }

        if (flywheelDownPressed && !lastCircleFlywheel) {
            targetVelocityRad -= 10.0;
        }

        targetVelocityRad = Range.clip(targetVelocityRad, 200.0, 1000);

        lastSquareFlywheel = flywheelUpPressed;
        lastCircleFlywheel = flywheelDownPressed;

        telemetry.addData("Manual Flywheel Target", targetVelocityRad);
    }

    private void handleAutoPark(Pose currentPose) {
        boolean parkTogglePressed = getAutoParkTogglePressed();

        if (parkTogglePressed && !lastTrianglePark) {
            if (!autoParkActive) {
                startAutoPark(currentPose);
            } else {
                cancelAutoPark();
            }
        }

        lastTrianglePark = parkTogglePressed;

        if (!autoParkActive) {
            return;
        }

        // Once the path first finishes, start a short settle timer.
        if (!autoParkSettling && !follower.isBusy()) {
            autoParkSettling = true;
            autoParkSettleTimer.reset();
        }

        // Stay in auto-park for a bit longer so the robot can settle.
        if (autoParkSettling) {
            if (autoParkSettleTimer.seconds() >= AUTO_PARK_SETTLE_TIME_SEC) {
                autoParkActive = false;
                autoParkSettling = false;
                follower.startTeleOpDrive();
            }
        }
    }

    private void startAutoPark(Pose currentPose) {
        Pose parkPose = getAutoParkPose();

        autoParkPath = new Path(
                new BezierLine(
                        currentPose,
                        parkPose
                )
        );

        autoParkPath.setLinearHeadingInterpolation(
                currentPose.getHeading(),
                parkPose.getHeading()
        );
        autoParkPath.setBrakingStrength(0.5);

        autoParkActive = true;
        autoParkSettling = false;
        autoParkSettleTimer.reset();

        follower.followPath(autoParkPath, true);

        if (oneDriver) {
            gamepad1.rumble(300);
        } else {
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }
    }

    private void cancelAutoPark() {
        autoParkActive = false;
        autoParkSettling = false;
        autoParkSettleTimer.reset();

        follower.startTeleOpDrive();
        follower.setTeleOpDrive(0, 0, 0, false);

        if (oneDriver) {
            gamepad1.rumble(500);
        } else {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }
    }

    private void drive(double robotHeadingDeg) {
        double trigger = Range.clip(1 - gamepad1.right_trigger, 0.2, 1);

        double forward = gamepad1.left_stick_y * trigger;
        double strafe = gamepad1.left_stick_x * trigger;
        double turn = -gamepad1.right_stick_x * trigger;

        // Mirror field-centric translation on red
        if (isRedAlliance) {
            forward = -forward;
            strafe = -strafe;
        }

        if (!(gamepad1.left_trigger > 0.5)) {
            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    turn,
                    false
            );
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * trigger,
                    -gamepad1.left_stick_x * trigger,
                    -gamepad1.right_stick_x * trigger,
                    true
            );
        }

        // Disable driver pose reset during zeroing only in one-driver mode.
        // In two-driver mode, keep it available since zeroing is on gamepad2.
        boolean allowDriverPoseReset = !oneDriver || !turretZeroingMode;

        if (gamepad1.circle && allowDriverPoseReset) {
            Pose resetPose = AllianceMirror.mirrorPose(
                    new Pose(24, 125.2, Math.toRadians(141)),
                    isRedAlliance
            );
            follower.setPose(resetPose);
            gamepad1.rumble(500);
        }
    }

    private void handleShotModeToggle() {
        boolean rb1Pressed = gamepad1.left_bumper;
        boolean rb2Pressed = gamepad2.left_bumper;

        if (oneDriver) {
            if (rb1Pressed && !lastRightBumper1) {
                cycleShotMode();
                gamepad1.rumble(400);
            }
        } else {
            if (rb2Pressed && !lastRightBumper2) {
                cycleShotMode();
                gamepad2.rumble(400);
            }
        }

        lastRightBumper1 = rb1Pressed;
        lastRightBumper2 = rb2Pressed;
    }
    private void cycleShotMode() {
        switch (shotMode) {
            case AUTO_SWITCH:
                shotMode = ShotMode.CLOSE;
                break;

            case CLOSE:
                shotMode = ShotMode.FAR;
                break;

            case FAR:
                shotMode = ShotMode.AUTO_SWITCH;
                break;
        }
    }

    private void handleTurretZeroingMode() {
        boolean togglePressed = getZeroingTogglePressed();

        if (togglePressed && !lastTurretZeroingToggle) {
            if (!turretZeroingMode) {
                    turretZeroingMode = true;
                    turretZeroingTargetDeg = 0.0;

                    lastDpadLeft = false;
                    lastDpadRight = false;
                    lastSquareZero = false;
                    lastCircleZero = false;

                    if (oneDriver) {
                        gamepad1.rumble(250);
                    } else {
                        gamepad2.rumble(250);
                    }
            } else {
                turret.zeroTurret();
                turretZeroingMode = false;
                turretZeroingTargetDeg = 0.0;

                lastDpadLeft = false;
                lastDpadRight = false;
                lastSquareZero = false;
                lastCircleZero = false;

                if (oneDriver) {
                    gamepad1.rumble(500);
                } else {
                    gamepad2.rumble(500);
                }
            }
        }

        lastTurretZeroingToggle = togglePressed;

        if (!turretZeroingMode) {
            return;
        }

        boolean leftPressed = oneDriver ? gamepad1.dpad_left : gamepad2.dpad_left;
        boolean rightPressed = oneDriver ? gamepad1.dpad_right : gamepad2.dpad_right;
        boolean squarePressed = oneDriver ? gamepad1.square : gamepad2.square;
        boolean circlePressed = oneDriver ? gamepad1.circle : gamepad2.circle;

        if (leftPressed && !lastDpadLeft) {
            turretZeroingTargetDeg += TURRET_ZERO_STEP_DEG;
        }

        if (rightPressed && !lastDpadRight) {
            turretZeroingTargetDeg -= TURRET_ZERO_STEP_DEG;
        }

        if (squarePressed && !lastSquareZero) {
            turretZeroingTargetDeg += TURRET_ZERO_BIG_STEP_DEG;
        }

        if (circlePressed && !lastCircleZero) {
            turretZeroingTargetDeg -= TURRET_ZERO_BIG_STEP_DEG;
        }

        lastDpadLeft = leftPressed;
        lastDpadRight = rightPressed;
        lastSquareZero = squarePressed;
        lastCircleZero = circlePressed;

        turretZeroingTargetDeg = Range.clip(
                turretZeroingTargetDeg,
                TURRET_MIN_DEG,
                TURRET_MAX_DEG
        );

        turret.setAngle(turretZeroingTargetDeg);

        telemetry.addLine("TURRET ZEROING MODE");
        telemetry.addData("Zeroing Target (deg)", turretZeroingTargetDeg);
        telemetry.addData("Current Turret Angle (deg)", turret.getCurrentAngle());
        telemetry.addData("Small Step", TURRET_ZERO_STEP_DEG);
        telemetry.addData("Big Step", TURRET_ZERO_BIG_STEP_DEG);
        telemetry.addLine("D-pad left/right = small move");
        telemetry.addLine("Square/circle = big move");
        telemetry.addLine("Press D-pad Up again to save zero and resume tracking");
    }

    private void handleTurretOffsetAdjustment() {
        boolean leftPressed;
        boolean rightPressed;

        if (oneDriver) {
            leftPressed = gamepad1.dpad_left;
            rightPressed = gamepad1.dpad_right;
        } else {
            leftPressed = gamepad2.dpad_left;
            rightPressed = gamepad2.dpad_right;
        }

        if (leftPressed && !lastDpadLeft) {
            turretAngleOffsetDeg += 1.0;
        }

        if (rightPressed && !lastDpadRight) {
            turretAngleOffsetDeg -= 1.0;
        }

        lastDpadLeft = leftPressed;
        lastDpadRight = rightPressed;
    }

    private void handleXToggle() {
        boolean xPressed;
        if (oneDriver) {
            xPressed = gamepad1.x;
        } else {
            xPressed = gamepad2.x;
        }

        if (xPressed && !lastX) {
            intakeToggleOn = !intakeToggleOn;

            if (feedState == FeedState.IDLE || feedState == FeedState.DONE) {
                if (intakeToggleOn) {
                    feeder.clutchOut();
                    feeder.armBlock();
                    intake.setPower(1.0);
                } else {
                    intake.setPower(0.0);
                }
            }
        }

        lastX = xPressed;
    }

    private void handleBSequence() {
        boolean bPressed;
        if (oneDriver) {
            bPressed = gamepad1.right_bumper;
        } else {
            bPressed = gamepad2.right_bumper;
        }

        if (bPressed && !lastB) {
            intakeToggleOn = false;
            intake.setPower(0.0);

            feeder.armShoot();
            feeder.clutchIn();

            sequenceTimer.reset();
            feedState = FeedState.WAIT_BEFORE_INTAKE;
        }

        lastB = bPressed;
    }

    private void updateFeedSequence() {
        switch (feedState) {
            case IDLE:
                break;

            case WAIT_BEFORE_INTAKE:
                if (sequenceTimer.seconds() >= 0.1) {
                    intake.setPower(1.0);
                    sequenceTimer.reset();
                    feedState = FeedState.RUN_INTAKE;
                }
                break;

            case RUN_INTAKE:
                if (sequenceTimer.seconds() >= 0.9) {
                    intake.setPower(1);
                    feeder.clutchOut();
                    feeder.armBlock();
                    feedState = FeedState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    private void trackGoalFromOdometry(Pose pose, double robotAngularVelRadPerSec) {
        double targetX = getTargetX();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double turretX = robotX - TURRET_CENTER_OFFSET_IN * Math.cos(robotHeadingRad);
        double turretY = robotY - TURRET_CENTER_OFFSET_IN * Math.sin(robotHeadingRad);

        double rx = turretX - robotX;
        double ry = turretY - robotY;

        double actualDx = targetX - turretX;
        double actualDy = TARGET_Y - turretY;
        double actualDistance = Math.hypot(actualDx, actualDy);

        double shotTimeSec = estimateShotTimeSec(actualDistance);

        double ux = 0.0;
        double uy = 0.0;
        if (actualDistance > 1e-6) {
            ux = actualDx / actualDistance;
            uy = actualDy / actualDistance;
        }

        double rotVx = -robotAngularVelRadPerSec * ry;
        double rotVy = robotAngularVelRadPerSec * rx;

        double shooterVx = fieldVxInPerSec + rotVx;
        double shooterVy = fieldVyInPerSec + rotVy;

        double compensatedTargetX = targetX;
        double compensatedTargetY = TARGET_Y;

        if (ENABLE_SHOT_ON_MOVE_COMP) {
            compensatedTargetX = targetX - shooterVx * shotTimeSec * TIME_TUNER;
            compensatedTargetY = TARGET_Y - shooterVy * shotTimeSec * TIME_TUNER;
        }

        double dx = compensatedTargetX - turretX;
        double dy = compensatedTargetY - turretY;

        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double angleToTargetRobotDeg = normalize180(angleToTargetFieldDeg - robotHeadingDeg);

        double tunableOffset = AllianceMirror.maybeMirrorTunableTurretOffset(
                turretAngleOffsetDeg, isRedAlliance
        );
        double baseOffset = AllianceMirror.maybeMirrorBaseTurretOffset(
                baseTurretAngleOffsetDeg, isRedAlliance
        );

        double desiredTurretDeg = normalize180(
                angleToTargetRobotDeg
                        + TURRET_OFFSET_DEG
                        + tunableOffset
                        + baseOffset
        );

        radialVelocityToGoal = shooterVx * ux + shooterVy * uy;

        predictedShotDistance = Math.hypot(dx, dy);
        predictedShotDistance = Math.max(0.0, predictedShotDistance);

        if (!predictedDistanceInitialized) {
            filteredPredictedShotDistance = predictedShotDistance;
            predictedDistanceInitialized = true;
        } else {
            filteredPredictedShotDistance =
                    PREDICTED_DISTANCE_ALPHA * predictedShotDistance
                            + (1.0 - PREDICTED_DISTANCE_ALPHA) * filteredPredictedShotDistance;
        }

        double safeTurretDeg = wrapIntoTurretWindow(
                desiredTurretDeg,
                turret.getCurrentAngle(),
                TURRET_MIN_DEG,
                TURRET_MAX_DEG
        );

        updateDesiredTurretVelocity(safeTurretDeg);

        turret.setTargetState(safeTurretDeg, desiredTurretVelDegPerSec);

        telemetry.addData("Target X", targetX);
        telemetry.addData("Turret Center X", turretX);
        telemetry.addData("Turret Center Y", turretY);
        telemetry.addData("Actual Dist To Goal", actualDistance);
        telemetry.addData("Shot Time (s)", shotTimeSec);
        telemetry.addData("Robot Omega (rad/s)", robotAngularVelRadPerSec);
        telemetry.addData("Rot Vel X", rotVx);
        telemetry.addData("Rot Vel Y", rotVy);
        telemetry.addData("Shooter Vel X", shooterVx);
        telemetry.addData("Shooter Vel Y", shooterVy);
        telemetry.addData("Radial Vel To Goal", radialVelocityToGoal);
        telemetry.addData("Predicted Shot Dist", predictedShotDistance);
        telemetry.addData("Filtered Pred Shot Dist", filteredPredictedShotDistance);
        telemetry.addData("Comp Target X", compensatedTargetX);
        telemetry.addData("Comp Target Y", compensatedTargetY);
        telemetry.addData("Angle To Goal Field", angleToTargetFieldDeg);
        telemetry.addData("Angle To Goal Robot", angleToTargetRobotDeg);
        telemetry.addData("Turret Target Deg", safeTurretDeg);
        telemetry.addData("Turret Target Vel (deg/s)", desiredTurretVelDegPerSec);
    }

    private void initVelocityEstimator(Pose pose) {
        lastPoseX = pose.getX();
        lastPoseY = pose.getY();
        fieldVxInPerSec = 0.0;
        fieldVyInPerSec = 0.0;
        velocityInitialized = true;
        poseVelocityTimer.reset();
    }

    private void updateEstimatedFieldVelocity(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();

        if (!velocityInitialized) {
            initVelocityEstimator(pose);
            return;
        }

        double dt = poseVelocityTimer.seconds();
        poseVelocityTimer.reset();

        if (dt <= 1e-4) {
            lastPoseX = x;
            lastPoseY = y;
            return;
        }

        double rawVx = (x - lastPoseX) / dt;
        double rawVy = (y - lastPoseY) / dt;

        fieldVxInPerSec = VELOCITY_FILTER_ALPHA * rawVx + (1.0 - VELOCITY_FILTER_ALPHA) * fieldVxInPerSec;
        fieldVyInPerSec = VELOCITY_FILTER_ALPHA * rawVy + (1.0 - VELOCITY_FILTER_ALPHA) * fieldVyInPerSec;

        lastPoseX = x;
        lastPoseY = y;
    }

    private double estimateShotTimeSec(double distanceInches) {
        return 0.77;
    }

    private double normalize180(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
    }

    private void updateDesiredTurretVelocity(double desiredTurretDeg) {
        if (!turretTargetVelInitialized) {
            lastDesiredTurretDeg = desiredTurretDeg;
            desiredTurretVelDegPerSec = 0.0;
            turretTargetVelInitialized = true;
            turretTargetVelTimer.reset();
            return;
        }

        double dt = turretTargetVelTimer.seconds();
        turretTargetVelTimer.reset();

        if (dt <= 1e-4) {
            lastDesiredTurretDeg = desiredTurretDeg;
            return;
        }

        double deltaDeg = desiredTurretDeg - lastDesiredTurretDeg;

        while (deltaDeg > 180.0) deltaDeg -= 360.0;
        while (deltaDeg < -180.0) deltaDeg += 360.0;

        double rawVelDegPerSec = deltaDeg / dt;

        desiredTurretVelDegPerSec =
                TURRET_TARGET_VEL_FILTER_ALPHA * rawVelDegPerSec
                        + (1.0 - TURRET_TARGET_VEL_FILTER_ALPHA) * desiredTurretVelDegPerSec;

        lastDesiredTurretDeg = desiredTurretDeg;
    }

    private double wrapIntoTurretWindow(double desiredDeg, double referenceDeg, double minDeg, double maxDeg) {
        double best = Double.NaN;

        for (int k = -2; k <= 2; k++) {
            double candidate = desiredDeg + 360.0 * k;
            if (candidate >= minDeg && candidate <= maxDeg) {
                if (Double.isNaN(best) || Math.abs(candidate - referenceDeg) < Math.abs(best - referenceDeg)) {
                    best = candidate;
                }
            }
        }

        if (Double.isNaN(best)) {
            best = Range.clip(desiredDeg, minDeg, maxDeg);
        }

        return best;
    }

    private void handleHoodAdjustment() {
        boolean dpadUpPressed = gamepad1.dpad_up;
        boolean dpadDownPressed = gamepad1.dpad_down;

        if (dpadUpPressed && !lastDpadUp) {
            hoodAngleDeg += 1.0;
            hoodAngleDeg = hood.setAngle(hoodAngleDeg);
        }

        if (dpadDownPressed && !lastDpadDown) {
            hoodAngleDeg -= 1.0;
            hoodAngleDeg = hood.setAngle(hoodAngleDeg);
        }

        lastDpadUp = dpadUpPressed;
        lastDpadDown = dpadDownPressed;

        telemetry.addData("Hood Angle (deg)", hoodAngleDeg);
    }

    private void updateShotFromDistance(double distance) {
        double[][] shotTable = {
                {37.0, 30.0, 267.0},
                {43.0, 30.0, 267.0},
                {50.0, 37.0, 277.0 + 5},
                {57.0, 37.0, 282.0 + 5},
                {63.5, 37.0, 292.0 + 5},
                {71.0, 39.0, 307.0 + 5},
                {77.0, 40.0, 312.0 + 5},
                {82.0, 42.0, 327.0 + 5},
                {88.0, 43.0, 332.0 + 5},
                {93.0, 44.0, 347.0 + 5},
                {99.0, 46.0, 364.0 + 5},
                {104.0, 47.0, 374.0 + 5},
                {110.0, 48.0, 389.0 + 5},
                {122.0, 53.0, 409.5 + 5}
        };

//        double[][] shotTable = {
//                {38.4, 30.0, 305.0},
//                {46.0, 34.0, 305.0},
//                {50.45, 35.0, 305.0},
//                {56.5, 38.0, 325.0},
//                {63.0, 39.0, 335.0},
//                {69.3, 40.0, 345.0},
//                {74.0, 41.0, 355.0},
//                {79.0, 44.0, 375.0},
//                {83.3, 44.0, 385.0},
//                {92.0, 46.0, 400.0},
//                {95.0, 47.0, 407.0},
//                {100.0, 47.0, 415.0},
//                {106.0, 49.0, 435.0},
//                {114.0, 50.0, 465.0},
//                {125.0, 51.0, 535.0}
//        };

        if (distance <= shotTable[0][0]) {
            hoodAngleDeg = shotTable[0][1];
            targetVelocityRad = shotTable[0][2];
            hoodAngleDeg = hood.setAngle(hoodAngleDeg);
            return;
        }

        int last = shotTable.length - 1;
        if (distance >= shotTable[last][0]) {
            hoodAngleDeg = shotTable[last][1];
            targetVelocityRad = shotTable[last][2];
            hoodAngleDeg = hood.setAngle(hoodAngleDeg);
            return;
        }

        for (int i = 0; i < shotTable.length - 1; i++) {
            double d1 = shotTable[i][0];
            double a1 = shotTable[i][1];
            double v1 = shotTable[i][2];

            double d2 = shotTable[i + 1][0];
            double a2 = shotTable[i + 1][1];
            double v2 = shotTable[i + 1][2];

            if (distance >= d1 && distance <= d2) {
                double t = (distance - d1) / (d2 - d1);

                hoodAngleDeg = a1 + t * (a2 - a1);
                targetVelocityRad = v1 + t * (v2 - v1) + 8;

                hoodAngleDeg = hood.setAngle(hoodAngleDeg);
                return;
            }
        }
    }
}