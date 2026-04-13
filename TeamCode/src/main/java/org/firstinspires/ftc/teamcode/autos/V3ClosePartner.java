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

import org.firstinspires.ftc.teamcode.AllianceMirror;
import org.firstinspires.ftc.teamcode.AllianceStore;
import org.firstinspires.ftc.teamcode.AutoStartStore;
import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "V3 Auto FSM Triple Gate Corrected")
public class V3ClosePartner extends LinearOpMode {

    private Servo hoodServo;
    private Servo armServo;
    private Servo clutchServo;

    private Follower follower;
    private Turret turret;
    private DcMotorEx intakeMotor;
    private FlywheelASG flywheel;

    private boolean isRedAlliance = false;

    // Starting pose (blue-native)
    private static final double START_X = 20.75;
    private static final double START_Y = 128.1;
    private static final double START_HEADING_DEG = -39.38;

    // First shot fixed setup
    private static final double FIRST_SHOT_HOOD_DEG = 37;
    private static final double FIRST_SHOT_FLYWHEEL_RAD = 310;

    // Goal position in field coordinates (blue-native)
    private static final double BLUE_TARGET_X = 5.0;
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

    // Original middle line cycle
    private Path toLine2;
    private Path backToShoot;

    // Gate paths
    private Path toGateOpenGate;
    private Path toGateIntake;
    private Path gateExtraIntakeMove;
    private Path backToShootFromGate;
    private Path gateExtraIntakeMoveAgain;
    private Path backToShootFromGateAgain;

    // Original higher-Y closer line cycle
    private Path toFourthPickup;
    private Path backToFinalShoot;

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

        // Original middle line cycle
        DRIVE_TO_LINE2,
        DRIVE_BACK_TO_SHOOT,
        SHOOT_SECOND,

        // Gate cycle #1
        DRIVE_TO_GATE_1,
        WAIT_AT_GATE_1,
        WAIT_FOR_GATE_INTAKE_1,
        DRIVE_EXTRA_INTAKE_AT_GATE_1,
        DRIVE_BACK_TO_SHOOT_THIRD,
        SHOOT_THIRD,

        // Gate cycle #2
        DRIVE_TO_GATE_2,
        WAIT_AT_GATE_2,
        WAIT_FOR_GATE_INTAKE_2,
        DRIVE_EXTRA_INTAKE_AT_GATE_2,
        DRIVE_BACK_TO_SHOOT_FOURTH,
        SHOOT_FOURTH,

        // Gate cycle #3
        DRIVE_TO_GATE_3,
        WAIT_AT_GATE_3,
        WAIT_FOR_GATE_INTAKE_3,
        DRIVE_EXTRA_INTAKE_AT_GATE_3,
        DRIVE_BACK_TO_SHOOT_FIFTH,
        SHOOT_FIFTH,

        // Original higher-Y closer line
        DRIVE_TO_FINAL_LINE,
        DRIVE_BACK_TO_FINAL_SHOOT,
        SHOOT_FINAL,

        DONE
    }

    private AutoState autoState = AutoState.DRIVE_TO_FIRST_SHOT;

    @Override
    public void runOpMode() {
        isRedAlliance = AllianceStore.isRed(hardwareMap.appContext);
        AutoStartStore.setClose(hardwareMap.appContext);

        Pose blueStartPose = new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEG)
        );
        startPose = AllianceMirror.mirrorPose(blueStartPose, isRedAlliance);

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

        double blueFirstShotX = START_X + 35 * Math.cos(Math.toRadians(START_HEADING_DEG));
        double blueFirstShotY = START_Y + 55.0 * Math.sin(Math.toRadians(START_HEADING_DEG));
        firstShotPose = AllianceMirror.mirrorPose(
                new Pose(blueFirstShotX, blueFirstShotY, 0),
                isRedAlliance
        );

        buildPaths();

        intakeMotor.setPower(0.0);
        flywheel.stop();
        clutchIn();
        armBlock();
        setHoodAngle(FIRST_SHOT_HOOD_DEG);
        turret.setAngle(90.0);

        telemetry.addLine("Initialized");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
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

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
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

            if (follower.isRobotStuck()) {
                follower.breakFollowing();
            }

            boolean turretShouldTrackGoal = autoState != AutoState.DONE;

            if (turretShouldTrackGoal) {
                if (enableDynamicShotControl) {
                    trackGoalFromOdometry(pose, enableShotOnMoveComp);

                    double lookupDistance = predictedDistanceInitialized
                            ? filteredPredictedShotDistance
                            : Math.hypot(getTargetX() - pose.getX(), TARGET_Y - pose.getY());

                    updateShotFromDistance(lookupDistance);
                } else {
                    trackGoalFromOdometry(pose, false);
                    setHoodAngle(FIRST_SHOT_HOOD_DEG);
                    targetVelocityRad = FIRST_SHOT_FLYWHEEL_RAD;
                }
            } else {
                turret.setAngle(0);
            }

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            updateFeedSequence();
            updateAutoState();

            turret.update();

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
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

    private double getTargetX() {
        return AllianceMirror.mirrorX(BLUE_TARGET_X, isRedAlliance);
    }

    private Pose p(double x, double y) {
        return AllianceMirror.mirrorPose(new Pose(x, y, 0), isRedAlliance);
    }

    private Pose p(double x, double y, double headingDeg) {
        return AllianceMirror.mirrorPose(
                new Pose(x, y, Math.toRadians(headingDeg)),
                isRedAlliance
        );
    }

    private double h(double headingDeg) {
        return Math.toRadians(AllianceMirror.mirrorHeadingDeg(headingDeg, isRedAlliance));
    }

    private void buildPaths() {
        toFirstShot = new Path(
                new BezierLine(startPose, firstShotPose)
        );
        toFirstShot.setTangentHeadingInterpolation();

        // ORIGINAL MIDDLE LINE
        toLine2 = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(48.0, 67.0),
                        p(40.0, 63.0),
                        p(12.5, 62.0)
                )
        );
        toLine2.setConstantHeadingInterpolation(h(180));

        backToShoot = new Path(
                new BezierCurve(
                        p(11.0, 65.0),
                        p(30.0, 65.0),
                        firstShotPose
                )
        );
        backToShoot.reverseHeadingInterpolation();

        // GATE
        toGateOpenGate = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(50.0, 66.326),
                        p(15.0, 67.5)
                )
        );
        toGateOpenGate.setLinearHeadingInterpolation(h(235), h(170), 0.9);

        toGateIntake = new Path(
                new BezierLine(
                        p(16.0, 69.5),
                        p(11.0, 59.0)
                )
        );
        toGateIntake.setConstantHeadingInterpolation(h(135));

        // ORIGINAL HIGHER-Y CLOSER LINE
        toFourthPickup = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(40.0, 84.0),
                        p(20.0, 84.0)
                )
        );
        toFourthPickup.setConstantHeadingInterpolation(h(180));

        backToFinalShoot = new Path(
                new BezierLine(
                        p(20.0, 84.0),
                        firstShotPose
                )
        );
        backToFinalShoot.reverseHeadingInterpolation();
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

            // ORIGINAL MIDDLE LINE CYCLE
            case DRIVE_TO_LINE2:
                if (follower.getCurrentTValue() > 0.35) {
                    follower.setMaxPower(0.8);
                    toLine2.setConstantHeadingInterpolation(h(180));
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
                    startGateCycle1();
                }
                break;

            // GATE CYCLE #1
            case DRIVE_TO_GATE_1:
                intakeMotor.setPower(0);
                if (!follower.isBusy()) {
                    autoTimer.reset();
                    autoState = AutoState.WAIT_AT_GATE_1;
                }
                break;

            case WAIT_AT_GATE_1:
                intakeMotor.setPower(0.0);
                if (autoTimer.seconds() >= 0.1) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, false);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_1;
                }
                break;

            case WAIT_FOR_GATE_INTAKE_1:
                intakeMotor.setPower(follower.getCurrentTValue() >= 0.5 ? 1.0 : 0.0);
                if (autoTimer.seconds() >= 1.5) {
                    startExtraGateIntakeMove1();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE_1:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= 1) {
                    startReturnFromGateToShoot1();
                }
                break;

            case DRIVE_BACK_TO_SHOOT_THIRD:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_THIRD;
                }
                break;

            case SHOOT_THIRD:
                if (feedState == FeedState.DONE) {
                    armBlock();
                    clutchOut();
                    startGateCycle2();
                }
                break;

            // GATE CYCLE #2
            case DRIVE_TO_GATE_2:
                intakeMotor.setPower(0);
                if (!follower.isBusy()) {
                    autoTimer.reset();
                    autoState = AutoState.WAIT_AT_GATE_2;
                }
                break;

            case WAIT_AT_GATE_2:
                intakeMotor.setPower(0.0);
                if (autoTimer.seconds() >= 0.25) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_2;
                }
                break;

            case WAIT_FOR_GATE_INTAKE_2:
                intakeMotor.setPower(follower.getCurrentTValue() >= 0.5 ? 1.0 : 0.0);
                if (autoTimer.seconds() >= 1.3) {
                    startExtraGateIntakeMove2();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE_2:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= 1) {
                    startReturnFromGateToShoot2();
                }
                break;

            case DRIVE_BACK_TO_SHOOT_FOURTH:
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
                    startGateCycle3();
                }
                break;

            // GATE CYCLE #3
            case DRIVE_TO_GATE_3:
                intakeMotor.setPower(0);
                if (!follower.isBusy()) {
                    autoTimer.reset();
                    autoState = AutoState.WAIT_AT_GATE_3;
                }
                break;

            case WAIT_AT_GATE_3:
                intakeMotor.setPower(0.0);
                if (autoTimer.seconds() >= 0.25) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_3;
                }
                break;

            case WAIT_FOR_GATE_INTAKE_3:
                intakeMotor.setPower(follower.getCurrentTValue() >= 0.5 ? 1.0 : 0.0);
                if (autoTimer.seconds() >= 1.3) {
                    startExtraGateIntakeMove3();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE_3:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= 1) {
                    startReturnFromGateToShoot3();
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
                    startFinalLineCycle();
                }
                break;

            // ORIGINAL HIGHER-Y CLOSER LINE
            case DRIVE_TO_FINAL_LINE:
                if (!follower.isBusy()) {
                    startReturnToFinalShoot();
                }
                break;

            case DRIVE_BACK_TO_FINAL_SHOOT:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_FINAL;
                }
                break;

            case SHOOT_FINAL:
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
                turret.setAngle(0);
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

    private void startGateCycle1() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toGateOpenGate, false);
        autoState = AutoState.DRIVE_TO_GATE_1;
    }

    private void startGateCycle2() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toGateOpenGate, false);
        autoState = AutoState.DRIVE_TO_GATE_2;
    }

    private void startGateCycle3() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toGateOpenGate, false);
        autoState = AutoState.DRIVE_TO_GATE_3;
    }

    private void startExtraGateIntakeMove1() {
        follower.setMaxPower(1.0);

        Pose currentPose = follower.getPose();
        gateExtraIntakeMove = new Path(
                new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY()),
                        new Pose(currentPose.getX(), currentPose.getY() + EXTRA_GATE_INTAKE_Y_IN)
                )
        );
        gateExtraIntakeMove.setConstantHeadingInterpolation(h(135));

        follower.followPath(gateExtraIntakeMove, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_1;
    }

    private void startReturnFromGateToShoot1() {
        follower.setMaxPower(1.0);

        backToShootFromGate = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        p(42.0, 59.0),
                        firstShotPose
                )
        );
        backToShootFromGate.reverseHeadingInterpolation();

        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.followPath(backToShootFromGate, true);
        autoState = AutoState.DRIVE_BACK_TO_SHOOT_THIRD;
    }

    private void startExtraGateIntakeMove2() {
        follower.setMaxPower(1.0);

        Pose currentPose = follower.getPose();
        gateExtraIntakeMoveAgain = new Path(
                new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY()),
                        new Pose(currentPose.getX(), currentPose.getY() + EXTRA_GATE_INTAKE_Y_IN)
                )
        );
        gateExtraIntakeMoveAgain.setConstantHeadingInterpolation(h(135));

        follower.followPath(gateExtraIntakeMoveAgain, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_2;
    }

    private void startReturnFromGateToShoot2() {
        follower.setMaxPower(1.0);

        backToShootFromGateAgain = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        p(42.0, 62.0),
                        firstShotPose
                )
        );
        backToShootFromGateAgain.reverseHeadingInterpolation();

        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.followPath(backToShootFromGateAgain, true);
        autoState = AutoState.DRIVE_BACK_TO_SHOOT_FOURTH;
    }

    private void startExtraGateIntakeMove3() {
        follower.setMaxPower(1.0);

        Pose currentPose = follower.getPose();
        gateExtraIntakeMoveAgain = new Path(
                new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY()),
                        new Pose(currentPose.getX(), currentPose.getY() + EXTRA_GATE_INTAKE_Y_IN)
                )
        );
        gateExtraIntakeMoveAgain.setConstantHeadingInterpolation(h(135));

        follower.followPath(gateExtraIntakeMoveAgain, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_3;
    }

    private void startReturnFromGateToShoot3() {
        follower.setMaxPower(1.0);

        backToShootFromGateAgain = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        p(42.0, 62.0),
                        firstShotPose
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

    private void startFinalLineCycle() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toFourthPickup, false);
        autoState = AutoState.DRIVE_TO_FINAL_LINE;
    }

    private void startReturnToFinalShoot() {
        intakeMotor.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(backToFinalShoot, false);
        autoState = AutoState.DRIVE_BACK_TO_FINAL_SHOOT;
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
        double targetX = getTargetX();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double fieldVxInPerSec = follower.getVelocity().getXComponent();
        double fieldVyInPerSec = follower.getVelocity().getYComponent();

        double turretX = robotX - TURRET_CENTER_OFFSET_IN * Math.cos(robotHeadingRad);
        double turretY = robotY - TURRET_CENTER_OFFSET_IN * Math.sin(robotHeadingRad);

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

        double compensatedTargetX = targetX;
        double compensatedTargetY = TARGET_Y;

        if (useShotOnMoveComp) {
            compensatedTargetX = targetX - fieldVxInPerSec * shotTimeSec;
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

        if (distance <= shotTable[0][0]) {
            hoodAngleDeg = shotTable[0][1];
            targetVelocityRad = shotTable[0][2];
            setHoodAngle(hoodAngleDeg);
            return;
        }

        int last = shotTable.length - 1;
        if (distance >= shotTable[last][0]) {
            hoodAngleDeg = shotTable[last][1];
            targetVelocityRad = shotTable[last][2];
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
                targetVelocityRad = (v1 + t * (v2 - v1)) + 15;
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
        armServo.setPosition(0.28);
    }

    public void armShoot() {
        armServo.setPosition(0.42);
    }

    public void setHoodAngle(double angleDeg) {
        final double MIN_ANGLE = 30.0;
        final double MAX_ANGLE = 60.0;

        final double MIN_POS = 0.42;
        final double MAX_POS = 0.95;

        hoodAngleDeg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDeg));

        double t = (hoodAngleDeg - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        double pos = MIN_POS + t * (MAX_POS - MIN_POS);

        pos = Math.max(MIN_POS, Math.min(MAX_POS, pos));

        hoodServo.setPosition(pos);
    }
}