package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "V3 Auto FSM")
public class V3Auto extends LinearOpMode {

    private Servo hoodServo;
    private Servo armServo;
    private Servo clutchServo;

    private Follower follower;
    private Turret turret;
    private DcMotorEx intakeMotor;
    private FlywheelASG flywheel;

    // Starting pose
    private static final double START_X = 20.75;
    private static final double START_Y = 128.1;
    private static final double START_HEADING_DEG = -39.38;

    // First shot fixed setup
    private static final double FIRST_SHOT_HOOD_DEG = 37;
    private static final double FIRST_SHOT_FLYWHEEL_RAD = 310;

    // Goal position in field coordinates
    private static final double TARGET_X = 5.0;
    private static final double TARGET_Y = 139.0;

    // Turret center offset from robot center (inches)
    private static final double TURRET_CENTER_OFFSET_IN = 1.5;

    // Turret limits / offset
    private static final double TURRET_MIN_DEG = -180.0;
    private static final double TURRET_MAX_DEG = 180.0;
    private static final double TURRET_OFFSET_DEG = 180.0;

    // Extra intake move after gate intake
    private static final double EXTRA_GATE_INTAKE_Y_IN = 4.5;

    // Dynamic shot control toggles
    private boolean enableDynamicShotControl = false;
    private boolean enableShotOnMoveComp = false;

    // Predicted distance smoothing
    private static final double PREDICTED_DISTANCE_ALPHA = 0.45;

    // Shot state
    private double hoodAngleDeg = 50.0;
    private double targetVelocityRad = 0.0;

    // Predicted distance state
    private double predictedShotDistance = 0.0;
    private double filteredPredictedShotDistance = 0.0;
    private boolean predictedDistanceInitialized = false;
    private double radialVelocityToGoal = 0.0;

    // Init button debounce
    private boolean lastCross = false;

    // Paths / poses
    private Pose startPose;
    private Pose firstShotPose;

    private Path toFirstShot;
    private Path toLine2;
    private Path backToShoot;

    private Path toGateOpenGate;
    private Path toGateIntake;
    private Path backToShootFromGate;
    private Path gateExtraIntakeMove;

    private Path toFourthPickup;
    private Path backToFinalShoot;

    private Path backToShootFromGateAgain;
    private Path gateExtraIntakeMoveAgain;

    private Path toLastLine;
    private Path backToShootFromLastLine;

    // Feed timer / state
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    private final ElapsedTime reverseTimer = new ElapsedTime();
    private boolean reversingIntake = false;

    private enum FeedState {
        IDLE,
        WAIT_BEFORE_INTAKE,
        RUN_INTAKE,
        DONE
    }

    private FeedState feedState = FeedState.IDLE;

    private enum AutoState {
        DRIVE_TO_FIRST_SHOT,
        WAIT_FOR_FIRST_SHOT_TO_FINISH,

        DRIVE_TO_LINE2,
        DRIVE_BACK_TO_SHOOT,
        SHOOT_SECOND,

        DRIVE_TO_GATE,
        WAIT_AT_GATE,
        WAIT_FOR_GATE_INTAKE,
        DRIVE_EXTRA_INTAKE_AT_GATE,
        DRIVE_BACK_TO_SHOOT_THIRD,
        SHOOT_THIRD,

        DRIVE_TO_FOURTH_PICKUP,
        DRIVE_BACK_TO_FINAL_SHOOT,
        SHOOT_FOURTH,

        DRIVE_TO_GATE_AGAIN,
        WAIT_AT_GATE_AGAIN,
        WAIT_FOR_GATE_INTAKE_AGAIN,
        DRIVE_EXTRA_INTAKE_AT_GATE_AGAIN,
        DRIVE_BACK_TO_SHOOT_FIFTH,
        SHOOT_FIFTH,

        DRIVE_TO_LAST_LINE,
        DRIVE_BACK_TO_SHOOT_LAST,
        SHOOT_LAST,

        DONE
    }

    private AutoState autoState = AutoState.DRIVE_TO_FIRST_SHOT;

    @Override
    public void runOpMode() {
        startPose = new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEG)
        );

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clutchServo = hardwareMap.get(Servo.class, "clutchServo");

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new FlywheelASG(hardwareMap, battery);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.updatePose();
        follower.setMaxPower(1.0);

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.REVERSE);

        double firstShotX = START_X + 35 * Math.cos(Math.toRadians(START_HEADING_DEG));
        double firstShotY = START_Y + 55.0 * Math.sin(Math.toRadians(START_HEADING_DEG));
        firstShotPose = new Pose(firstShotX, firstShotY);

        buildPaths();

        intakeMotor.setPower(0.0);
        flywheel.stop();
        clutchIn();
        armBlock();
        setHoodAngle(FIRST_SHOT_HOOD_DEG);
        turret.setAngle(90.0);

        telemetry.addLine("Initialized");
        telemetry.addData("Start Pose", startPose);
        telemetry.addData("First Shot Pose", firstShotPose);
        telemetry.addLine("Press cross in init to re-zero starting pose");
        telemetry.update();

        while (opModeInInit()) {
            boolean crossPressed = gamepad1.cross;
            if (crossPressed && !lastCross) {
                follower.setStartingPose(startPose);
                follower.updatePose();
            }
            lastCross = crossPressed;

            turret.setAngle(90.0);
            turret.update();

            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Turret Target", 90.0);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        hoodAngleDeg = FIRST_SHOT_HOOD_DEG;
        targetVelocityRad = FIRST_SHOT_FLYWHEEL_RAD;
        setHoodAngle(hoodAngleDeg);
        flywheel.setTargetVelocity(targetVelocityRad);

        follower.followPath(toFirstShot, false);
        autoState = AutoState.DRIVE_TO_FIRST_SHOT;

        while (opModeIsActive()) {
            follower.update();

            Pose pose = follower.getPose();

            if (enableDynamicShotControl) {
                trackGoalFromOdometry(pose, enableShotOnMoveComp);

                double lookupDistance = predictedDistanceInitialized
                        ? filteredPredictedShotDistance
                        : Math.hypot(TARGET_X - pose.getX(), TARGET_Y - pose.getY());

                updateShotFromDistance(lookupDistance);
            } else {
                trackGoalFromOdometry(pose, false);
                setHoodAngle(FIRST_SHOT_HOOD_DEG);
                targetVelocityRad = FIRST_SHOT_FLYWHEEL_RAD;
            }

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            updateFeedSequence();
            updateAutoState();

            turret.update();

            telemetry.addData("Auto State", autoState);
            telemetry.addData("Feed State", feedState);
            telemetry.addData("Pose", pose);
            telemetry.addData("Path T", follower.getCurrentTValue());
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Dynamic Shot", enableDynamicShotControl);
            telemetry.addData("Shot On Move", enableShotOnMoveComp);
            telemetry.addData("Hood Angle", hoodAngleDeg);
            telemetry.addData("Flywheel Target", targetVelocityRad);
            telemetry.addData("Flywheel Actual", flywheel.getVelocityRadPerSec());
            telemetry.addData("Predicted Distance", filteredPredictedShotDistance);
            telemetry.update();
        }

        flywheel.stop();
        intakeMotor.setPower(0.0);
    }

    private void buildPaths() {
        toFirstShot = new Path(
                new BezierLine(startPose, firstShotPose)
        );
        toFirstShot.setTangentHeadingInterpolation();

        toLine2 = new Path(
                new BezierCurve(
                        new Pose(firstShotPose.getX(), firstShotPose.getY()),
                        new Pose(48.0, 67.0),
                        new Pose(40.0, 63.0),
                        new Pose(12.5, 62.0)
                )
        );
        toLine2.setConstantHeadingInterpolation(Math.toRadians(180));

        backToShoot = new Path(
                new BezierCurve(
                        new Pose(11.0, 65.0),
                        new Pose(30.0, 65.0),
                        firstShotPose
                )
        );
        backToShoot.reverseHeadingInterpolation();

        toGateOpenGate = new Path(
                new BezierCurve(
                        new Pose(firstShotPose.getX(), firstShotPose.getY()),
                        new Pose(50.0, 66.326),
                        new Pose(16.0, 66.5)
                )
        );
        toGateOpenGate.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(165), 0.9);

        toGateIntake = new Path(
                new BezierLine(
                        new Pose(16.0, 69.5),
                        new Pose(11.0, 59.0)
                )
        );
        toGateIntake.setConstantHeadingInterpolation(Math.toRadians(135));

        toFourthPickup = new Path(
                new BezierCurve(
                        new Pose(firstShotPose.getX(), firstShotPose.getY()),
                        new Pose(40.0, 84.0),
                        new Pose(20.0, 84.0)
                )
        );
        toFourthPickup.setConstantHeadingInterpolation(Math.toRadians(180));

        backToFinalShoot = new Path(
                new BezierLine(
                        new Pose(20.0, 84.0),
                        new Pose(firstShotPose.getX(), firstShotPose.getY())
                )
        );
        backToFinalShoot.reverseHeadingInterpolation();

        toLastLine = new Path(
                new BezierCurve(
                        new Pose(firstShotPose.getX(), firstShotPose.getY()),
                        new Pose(48.0, 43.0),
                        new Pose(40.0, 39.0),
                        new Pose(12.5, 38.0)
                )
        );
        toLastLine.setTangentHeadingInterpolation();

        backToShootFromLastLine = new Path(
                new BezierCurve(
                        new Pose(11.0, 41.0),
                        new Pose(30.0, 41.0),
                        new Pose(firstShotPose.getX() + 10, firstShotPose.getY() + 18)
                )
        );
        backToShootFromLastLine.reverseHeadingInterpolation();
    }

    private void updateAutoState() {
        switch (autoState) {
            case DRIVE_TO_FIRST_SHOT:
                if (feedState == FeedState.IDLE && follower.getCurrentTValue() > 0.9) {
                    startFeedSequence();
                }

                if (!follower.isBusy()) {
                    if (feedState == FeedState.DONE) {
                        finishFirstShotAndStartLine2();
                    } else {
                        autoState = AutoState.WAIT_FOR_FIRST_SHOT_TO_FINISH;
                    }
                }
                break;

            case WAIT_FOR_FIRST_SHOT_TO_FINISH:
                if (feedState == FeedState.DONE) {
                    finishFirstShotAndStartLine2();
                }
                break;

            case DRIVE_TO_LINE2:
                if (follower.getCurrentTValue() > 0.35) {
                    follower.setMaxPower(0.8);
                    toLine2.setConstantHeadingInterpolation(Math.toRadians(180));
                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(backToShoot, true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT;

                    enableDynamicShotControl = true;
                    enableShotOnMoveComp = true;
                }
                break;

            case DRIVE_BACK_TO_SHOOT:
                follower.setMaxPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_SECOND;
                }
                break;

            case SHOOT_SECOND:
                if (feedState == FeedState.DONE) {
                    armBlock();
                    clutchOut();
                    startThirdCycleToGate();
                }
                break;

            case DRIVE_TO_GATE:
                if (!follower.isBusy()) {
                    autoTimer.reset();
                    autoState = AutoState.WAIT_AT_GATE;
                }
                break;

            case WAIT_AT_GATE:
                intakeMotor.setPower(1.0);
                if (autoTimer.seconds() >= 0.1) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, false);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE;
                }
                break;

            case WAIT_FOR_GATE_INTAKE:
                intakeMotor.setPower(1.0);
                if (autoTimer.seconds() >= 1.5) {
                    startExtraGateIntakeMove();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    startReturnFromGateToShoot();
                }
                break;

            case DRIVE_BACK_TO_SHOOT_THIRD:
                intakeMotor.setPower(1.0);
                if (follower.getCurrentTValue() > 0.3) {
                    backToShootFromGate.reverseHeadingInterpolation();
                }
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_THIRD;
                }
                break;

            case SHOOT_THIRD:
                if (feedState == FeedState.DONE) {
                    armBlock();
                    clutchOut();
                    startFourthPickupCycle();
                }
                break;

            case DRIVE_TO_FOURTH_PICKUP:
                if (!follower.isBusy()) {
                    startReturnToFinalShoot();
                }
                break;

            case DRIVE_BACK_TO_FINAL_SHOOT:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_FOURTH;
                }
                break;

            case SHOOT_FOURTH:
                if (feedState == FeedState.DONE) {
                    armBlock();
                    clutchOut();
                    startGateCycleAgain();
                }
                break;

            case DRIVE_TO_GATE_AGAIN:
                if (!follower.isBusy()) {
                    autoTimer.reset();
                    autoState = AutoState.WAIT_AT_GATE_AGAIN;
                }
                break;

            case WAIT_AT_GATE_AGAIN:
                intakeMotor.setPower(1.0);
                if (autoTimer.seconds() >= 0.2) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_AGAIN;
                }
                break;

            case WAIT_FOR_GATE_INTAKE_AGAIN:
                intakeMotor.setPower(1.0);
                if (autoTimer.seconds() >= 1.5) {
                    startExtraGateIntakeMoveAgain();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE_AGAIN:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    startReturnFromGateToShootAgain();
                }
                break;

            case DRIVE_BACK_TO_SHOOT_FIFTH:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_FIFTH;
                }
                break;

            case SHOOT_FIFTH:
                if (feedState == FeedState.DONE) {
                    armBlock();
                    clutchOut();
                    startLastLineCycle();
                }
                break;

            case DRIVE_TO_LAST_LINE:
                if (follower.getCurrentTValue() > 0.25) {
                    follower.setMaxPower(0.8);
                    toLastLine.setConstantHeadingInterpolation(Math.toRadians(180));
                }
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1.0);
                    follower.setMaxPower(1.0);
                    follower.followPath(backToShootFromLastLine, true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT_LAST;

                    enableDynamicShotControl = true;
                    enableShotOnMoveComp = true;
                }
                break;

            case DRIVE_BACK_TO_SHOOT_LAST:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_LAST;
                }
                break;

            case SHOOT_LAST:
                if (feedState == FeedState.DONE) {
                    intakeMotor.setPower(0.0);
                    armBlock();
                    clutchOut();
                    enableDynamicShotControl = false;
                    enableShotOnMoveComp = false;
                    turret.setAngle(0);
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                intakeMotor.setPower(0.0);
                break;
        }
    }

    private void finishFirstShotAndStartLine2() {
        clutchOut();
        armBlock();

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toLine2, false);
        autoState = AutoState.DRIVE_TO_LINE2;
    }

    private void startThirdCycleToGate() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toGateOpenGate, false);
        autoState = AutoState.DRIVE_TO_GATE;
    }

    private void startExtraGateIntakeMove() {
        follower.setMaxPower(1.0);

        Pose currentPose = follower.getPose();
        gateExtraIntakeMove = new Path(
                new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY()),
                        new Pose(currentPose.getX(), currentPose.getY() + EXTRA_GATE_INTAKE_Y_IN)
                )
        );
        gateExtraIntakeMove.setConstantHeadingInterpolation(Math.toRadians(135));

        follower.followPath(gateExtraIntakeMove, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE;
    }

    private void startReturnFromGateToShoot() {
        follower.setMaxPower(1.0);

        backToShootFromGate = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(42.0, 59),
                        new Pose(firstShotPose.getX(), firstShotPose.getY())
                )
        );
        backToShootFromGate.setConstantHeadingInterpolation(Math.toRadians(135));

        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.followPath(backToShootFromGate, true);
        autoState = AutoState.DRIVE_BACK_TO_SHOOT_THIRD;
    }

    private void startFourthPickupCycle() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toFourthPickup, false);
        autoState = AutoState.DRIVE_TO_FOURTH_PICKUP;
    }

    private void startReturnToFinalShoot() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(backToFinalShoot, true);
        autoState = AutoState.DRIVE_BACK_TO_FINAL_SHOOT;
    }

    private void startGateCycleAgain() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toGateOpenGate, false);
        autoState = AutoState.DRIVE_TO_GATE_AGAIN;
    }

    private void startExtraGateIntakeMoveAgain() {
        follower.setMaxPower(1.0);

        Pose currentPose = follower.getPose();
        gateExtraIntakeMoveAgain = new Path(
                new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY()),
                        new Pose(currentPose.getX(), currentPose.getY() + EXTRA_GATE_INTAKE_Y_IN)
                )
        );
        gateExtraIntakeMoveAgain.setConstantHeadingInterpolation(Math.toRadians(135));

        follower.followPath(gateExtraIntakeMoveAgain, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_AGAIN;
    }

    private void startReturnFromGateToShootAgain() {
        follower.setMaxPower(1.0);

        backToShootFromGateAgain = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        new Pose(42.0, 62.0),
                        new Pose(firstShotPose.getX(), firstShotPose.getY())
                )
        );
        backToShootFromGateAgain.reverseHeadingInterpolation();

        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.followPath(backToShootFromGateAgain, true);
        autoState = AutoState.DRIVE_BACK_TO_SHOOT_FIFTH;
    }

    private void startLastLineCycle() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toLastLine, false);
        autoState = AutoState.DRIVE_TO_LAST_LINE;
    }

    private void startFeedSequence() {
        armShoot();
        feedTimer.reset();
        feedState = FeedState.WAIT_BEFORE_INTAKE;
    }

    private void updateFeedSequence() {

        if (reversingIntake) {
            if (reverseTimer.seconds() < 0.25) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(1.0);
                reversingIntake = false;
            }
        }

        switch (feedState) {
            case IDLE:
                break;

            case WAIT_BEFORE_INTAKE:
                clutchIn();
                intakeMotor.setPower(0.0);

                if (feedTimer.seconds() >= 0.1) {
                    intakeMotor.setPower(1.0);
                    feedState = FeedState.RUN_INTAKE;
                }
                break;

            case RUN_INTAKE:
                if (feedTimer.seconds() >= 1.0) {
                    reversingIntake = true;
                    reverseTimer.reset();

                    intakeMotor.setPower(0.0);
                    feedState = FeedState.DONE;
                    clutchOut();
                }
                break;

            case DONE:
                break;
        }
    }

    private void trackGoalFromOdometry(Pose pose, boolean useShotOnMoveComp) {
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double fieldVxInPerSec = follower.getVelocity().getXComponent();
        double fieldVyInPerSec = follower.getVelocity().getYComponent();

        double turretX = robotX - TURRET_CENTER_OFFSET_IN * Math.cos(robotHeadingRad);
        double turretY = robotY - TURRET_CENTER_OFFSET_IN * Math.sin(robotHeadingRad);

        double actualDx = TARGET_X - turretX;
        double actualDy = TARGET_Y - turretY;
        double actualDistance = Math.hypot(actualDx, actualDy);

        double shotTimeSec = estimateShotTimeSec(actualDistance);

        double ux = 0.0;
        double uy = 0.0;
        if (actualDistance > 1e-6) {
            ux = actualDx / actualDistance;
            uy = actualDy / actualDistance;
        }

        radialVelocityToGoal = fieldVxInPerSec * ux + fieldVyInPerSec * uy;

        predictedShotDistance = actualDistance - radialVelocityToGoal * shotTimeSec;
        predictedShotDistance = Math.max(0.0, predictedShotDistance);

        if (!predictedDistanceInitialized) {
            filteredPredictedShotDistance = predictedShotDistance;
            predictedDistanceInitialized = true;
        } else {
            filteredPredictedShotDistance =
                    PREDICTED_DISTANCE_ALPHA * predictedShotDistance
                            + (1.0 - PREDICTED_DISTANCE_ALPHA) * filteredPredictedShotDistance;
        }

        double compensatedTargetX = TARGET_X;
        double compensatedTargetY = TARGET_Y;

        if (useShotOnMoveComp) {
            compensatedTargetX = TARGET_X - fieldVxInPerSec * shotTimeSec;
            compensatedTargetY = TARGET_Y - fieldVyInPerSec * shotTimeSec;
        }

        double dx = compensatedTargetX - turretX;
        double dy = compensatedTargetY - turretY;

        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double angleToTargetRobotDeg = normalize180(angleToTargetFieldDeg - robotHeadingDeg);
        double desiredTurretDeg = normalize180(angleToTargetRobotDeg + TURRET_OFFSET_DEG);

        double safeTurretDeg = wrapIntoTurretWindow(
                desiredTurretDeg,
                turret.getCurrentAngle(),
                TURRET_MIN_DEG,
                TURRET_MAX_DEG
        );

        turret.setAngle(safeTurretDeg);
    }

    private double estimateShotTimeSec(double distanceInches) {
        return 0.77;
    }

    private double normalize180(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
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

    private void updateShotFromDistance(double distance) {
        double[][] shotTable = {
                {47.0, 30.0, 275.0},
                {52.0, 30.0, 275.0},
                {59.0, 33.0, 280.0},
                {64.5, 36.0, 285.0},
                {70.0, 36.0, 300.0},
                {76.5, 36.0, 320.0},
                {81.0, 38.0, 320.0},
                {88.0, 38.0, 330.0},
                {94.0, 38.0, 340.0},
                {102.0, 40.0, 340.0},
                {115.0, 44.0, 375.0},
                {119.7, 44.0, 380.0},
                {126.0, 44.0, 385.0}
        };

        if (distance <= shotTable[0][0]) {
            hoodAngleDeg = shotTable[0][1];
            targetVelocityRad = shotTable[0][2] + 20.0;
            setHoodAngle(hoodAngleDeg);
            return;
        }

        int last = shotTable.length - 1;
        if (distance >= shotTable[last][0]) {
            hoodAngleDeg = shotTable[last][1];
            targetVelocityRad = shotTable[last][2] + 10.0;
            setHoodAngle(hoodAngleDeg);
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
                targetVelocityRad = (v1 + t * (v2 - v1)) + 10;
                setHoodAngle(hoodAngleDeg);
                return;
            }
        }
    }

    public void clutchIn() {
        clutchServo.setPosition(0.48);
    }

    public void clutchOut() {
        clutchServo.setPosition(0.52);
    }

    public void armBlock() {
        armServo.setPosition(0.39);
    }

    public void armShoot() {
        armServo.setPosition(0.54);
    }

    public void setHoodAngle(double angleDeg) {
        final double MIN_ANGLE = 30.0;
        final double MAX_ANGLE = 60.0;

        final double MIN_POS = 0.395;
        final double MAX_POS = 0.9;

        hoodAngleDeg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDeg));

        double t = (hoodAngleDeg - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        double pos = MIN_POS + t * (MAX_POS - MIN_POS);

        pos = Math.max(MIN_POS, Math.min(MAX_POS, pos));

        hoodServo.setPosition(pos);
    }
}