package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.RobotConfig;

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
import org.firstinspires.ftc.teamcode.autoshared.V3ClosePartnerConfig;
import org.firstinspires.ftc.teamcode.hardwareClasses.Flywheel;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "V3 Auto FSM Triple Gate Corrected")
public class V3ClosePartner extends LinearOpMode {

    private Servo hoodServo;
    private Servo armServo;
    private Servo clutchServo;

    private Follower follower;
    private Turret turret;
    private DcMotorEx intakeMotor;
    private Flywheel flywheel;

    private boolean isRedAlliance = false;

    // ===== Tunables / geometry — single source of truth in V3ClosePartnerConfig =====
    private static final double START_X = V3ClosePartnerConfig.START_X;
    private static final double START_Y = V3ClosePartnerConfig.START_Y;
    private static final double START_HEADING_DEG = V3ClosePartnerConfig.START_HEADING_DEG;

    private static final double FIRST_SHOT_HOOD_DEG = V3ClosePartnerConfig.FIRST_SHOT_HOOD_DEG;
    private static final double FIRST_SHOT_FLYWHEEL_RAD = V3ClosePartnerConfig.FIRST_SHOT_FLYWHEEL_RAD;

    private static final double BLUE_TARGET_X = V3ClosePartnerConfig.BLUE_TARGET_X;
    private static final double TARGET_Y = V3ClosePartnerConfig.TARGET_Y;

    private static final double TURRET_CENTER_OFFSET_IN = V3ClosePartnerConfig.TURRET_CENTER_OFFSET_IN;
    private static final double TURRET_MIN_DEG = V3ClosePartnerConfig.TURRET_MIN_DEG;
    private static final double TURRET_MAX_DEG = V3ClosePartnerConfig.TURRET_MAX_DEG;
    private static final double TURRET_OFFSET_DEG = V3ClosePartnerConfig.TURRET_OFFSET_DEG;

    private static final double EXTRA_GATE_INTAKE_Y_IN = V3ClosePartnerConfig.EXTRA_GATE_INTAKE_Y_IN;

    // Dynamic shot control toggles
    private boolean enableDynamicShotControl = false;
    private boolean enableShotOnMoveComp = false;

    private static final double PREDICTED_DISTANCE_ALPHA = V3ClosePartnerConfig.PREDICTED_DISTANCE_ALPHA;

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

        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo = hardwareMap.get(Servo.class, RobotConfig.HOOD_SERVO);
        armServo = hardwareMap.get(Servo.class, RobotConfig.FEEDER_ARM_SERVO);
        clutchServo = hardwareMap.get(Servo.class, RobotConfig.FEEDER_CLUTCH_SERVO);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new Flywheel(hardwareMap, battery);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.updatePose();
        follower.setMaxPower(1.0);

        turret = new Turret(hardwareMap, RobotConfig.TURRET_MOTOR, DcMotorSimple.Direction.REVERSE);

        double blueFirstShotX = START_X + V3ClosePartnerConfig.FIRST_SHOT_FWD_IN * Math.cos(Math.toRadians(START_HEADING_DEG));
        double blueFirstShotY = START_Y + V3ClosePartnerConfig.FIRST_SHOT_SIDE_IN * Math.sin(Math.toRadians(START_HEADING_DEG));
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

            turret.setAngle(V3ClosePartnerConfig.INIT_TURRET_ANGLE_DEG);
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
                        p(V3ClosePartnerConfig.L2_C1X, V3ClosePartnerConfig.L2_C1Y),
                        p(V3ClosePartnerConfig.L2_C2X, V3ClosePartnerConfig.L2_C2Y),
                        p(V3ClosePartnerConfig.L2_ENDX, V3ClosePartnerConfig.L2_ENDY)
                )
        );
        toLine2.setConstantHeadingInterpolation(h(V3ClosePartnerConfig.LINE_HEADING_DEG));

        backToShoot = new Path(
                new BezierCurve(
                        p(V3ClosePartnerConfig.BTS_STARTX, V3ClosePartnerConfig.BTS_STARTY),
                        p(V3ClosePartnerConfig.BTS_C1X, V3ClosePartnerConfig.BTS_C1Y),
                        firstShotPose
                )
        );
        backToShoot.reverseHeadingInterpolation();

        // GATE
        toGateOpenGate = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(V3ClosePartnerConfig.GATE_C1X, V3ClosePartnerConfig.GATE_C1Y),
                        p(V3ClosePartnerConfig.GATE_ENDX, V3ClosePartnerConfig.GATE_ENDY)
                )
        );
        toGateOpenGate.setLinearHeadingInterpolation(
                h(V3ClosePartnerConfig.GATE_OPEN_HEADING_START_DEG),
                h(V3ClosePartnerConfig.GATE_OPEN_HEADING_END_DEG),
                V3ClosePartnerConfig.GATE_OPEN_HEADING_T);

        toGateIntake = new Path(
                new BezierLine(
                        p(V3ClosePartnerConfig.GI_STARTX, V3ClosePartnerConfig.GI_STARTY),
                        p(V3ClosePartnerConfig.GI_ENDX, V3ClosePartnerConfig.GI_ENDY)
                )
        );
        toGateIntake.setConstantHeadingInterpolation(h(V3ClosePartnerConfig.GATE_INTAKE_HEADING_DEG));

        // ORIGINAL HIGHER-Y CLOSER LINE
        toFourthPickup = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(V3ClosePartnerConfig.FP_C1X, V3ClosePartnerConfig.FP_C1Y),
                        p(V3ClosePartnerConfig.FP_ENDX, V3ClosePartnerConfig.FP_ENDY)
                )
        );
        toFourthPickup.setConstantHeadingInterpolation(h(V3ClosePartnerConfig.LINE_HEADING_DEG));

        backToFinalShoot = new Path(
                new BezierLine(
                        p(V3ClosePartnerConfig.FP_ENDX, V3ClosePartnerConfig.FP_ENDY),
                        firstShotPose
                )
        );
        backToFinalShoot.reverseHeadingInterpolation();
    }

    private void updateAutoState() {
        switch (autoState) {
            case DRIVE_TO_FIRST_SHOT:
                if (feedState == FeedState.IDLE && follower.getCurrentTValue() > V3ClosePartnerConfig.FIRST_SHOT_FEED_TVALUE) {
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
                if (follower.getCurrentTValue() > V3ClosePartnerConfig.LINE2_SLOW_TVALUE) {
                    follower.setMaxPower(V3ClosePartnerConfig.LINE2_SLOW_POWER);
                    toLine2.setConstantHeadingInterpolation(h(V3ClosePartnerConfig.LINE_HEADING_DEG));
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
                if (autoTimer.seconds() >= V3ClosePartnerConfig.WAIT_AT_GATE_1_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, false);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_1;
                }
                break;

            case WAIT_FOR_GATE_INTAKE_1:
                intakeMotor.setPower(follower.getCurrentTValue() >= V3ClosePartnerConfig.GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.GATE_INTAKE_1_SEC) {
                    startExtraGateIntakeMove1();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE_1:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= V3ClosePartnerConfig.EXTRA_MOVE_TIMEOUT_SEC) {
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
                if (autoTimer.seconds() >= V3ClosePartnerConfig.WAIT_AT_GATE_2_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_2;
                }
                break;

            case WAIT_FOR_GATE_INTAKE_2:
                intakeMotor.setPower(follower.getCurrentTValue() >= V3ClosePartnerConfig.GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.GATE_INTAKE_2_SEC) {
                    startExtraGateIntakeMove2();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE_2:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= V3ClosePartnerConfig.EXTRA_MOVE_TIMEOUT_SEC) {
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
                if (autoTimer.seconds() >= V3ClosePartnerConfig.WAIT_AT_GATE_3_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_3;
                }
                break;

            case WAIT_FOR_GATE_INTAKE_3:
                intakeMotor.setPower(follower.getCurrentTValue() >= V3ClosePartnerConfig.GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= V3ClosePartnerConfig.GATE_INTAKE_3_SEC) {
                    startExtraGateIntakeMove3();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE_3:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= V3ClosePartnerConfig.EXTRA_MOVE_TIMEOUT_SEC) {
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
        gateExtraIntakeMove.setConstantHeadingInterpolation(h(V3ClosePartnerConfig.GATE_INTAKE_HEADING_DEG));

        follower.followPath(gateExtraIntakeMove, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_1;
    }

    private void startReturnFromGateToShoot1() {
        follower.setMaxPower(1.0);

        backToShootFromGate = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        p(V3ClosePartnerConfig.RG1_C1X, V3ClosePartnerConfig.RG1_C1Y),
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
        gateExtraIntakeMoveAgain.setConstantHeadingInterpolation(h(V3ClosePartnerConfig.GATE_INTAKE_HEADING_DEG));

        follower.followPath(gateExtraIntakeMoveAgain, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_2;
    }

    private void startReturnFromGateToShoot2() {
        follower.setMaxPower(1.0);

        backToShootFromGateAgain = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        p(V3ClosePartnerConfig.RG2_C1X, V3ClosePartnerConfig.RG2_C1Y),
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
        gateExtraIntakeMoveAgain.setConstantHeadingInterpolation(h(V3ClosePartnerConfig.GATE_INTAKE_HEADING_DEG));

        follower.followPath(gateExtraIntakeMoveAgain, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_3;
    }

    private void startReturnFromGateToShoot3() {
        follower.setMaxPower(1.0);

        backToShootFromGateAgain = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        p(V3ClosePartnerConfig.RG2_C1X, V3ClosePartnerConfig.RG2_C1Y),
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
            if (reverseTimer.seconds() < V3ClosePartnerConfig.REVERSE_TIME_SEC) {
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

                if (feedTimer.seconds() >= V3ClosePartnerConfig.FEED_START_DELAY_SEC) {
                    intakeMotor.setPower(1.0);
                    feedState = FeedState.RUN_INTAKE;
                }
                break;

            case RUN_INTAKE:
                if (feedTimer.seconds() >= V3ClosePartnerConfig.FEED_TOTAL_TIME_SEC) {
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
        return V3ClosePartnerConfig.SHOT_TIME_SEC;
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
        double[][] shotTable = V3ClosePartnerConfig.SHOT_TABLE;

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