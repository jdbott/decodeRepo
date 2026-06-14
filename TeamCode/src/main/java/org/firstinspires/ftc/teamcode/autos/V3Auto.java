package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.RobotConfig;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoStartStore;
import org.firstinspires.ftc.teamcode.autoshared.V3AutoConfig;
import org.firstinspires.ftc.teamcode.hardwareClasses.Feeder;
import org.firstinspires.ftc.teamcode.hardwareClasses.Flywheel;
import org.firstinspires.ftc.teamcode.hardwareClasses.Hood;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.AllianceMirror;
import org.firstinspires.ftc.teamcode.AllianceStore;

@Autonomous(name = "V3 Auto FSM")
public class V3Auto extends LinearOpMode {

    private Hood hood;
    private Feeder feeder;

    private Follower follower;
    private Turret turret;
    private Intake intake;
    private Flywheel flywheel;

    private boolean isRedAlliance = false;

    // ===== Tunables / geometry — single source of truth in V3AutoConfig =====
    private static final double START_X = V3AutoConfig.START_X;
    private static final double START_Y = V3AutoConfig.START_Y;
    private static final double START_HEADING_DEG = V3AutoConfig.START_HEADING_DEG;

    // First shot fixed setup
    private static double FIRST_SHOT_HOOD_DEG = V3AutoConfig.FIRST_SHOT_HOOD_DEG;
    private static double FIRST_SHOT_FLYWHEEL_RAD = V3AutoConfig.FIRST_SHOT_FLYWHEEL_RAD;

    // Goal position in field coordinates (blue-native)
    private static final double BLUE_TARGET_X = V3AutoConfig.BLUE_TARGET_X;
    private static final double TARGET_Y = V3AutoConfig.TARGET_Y;

    // Turret center offset from robot center (inches)
    private static final double TURRET_CENTER_OFFSET_IN = V3AutoConfig.TURRET_CENTER_OFFSET_IN;

    // Turret limits / offset
    private static final double TURRET_MIN_DEG = V3AutoConfig.TURRET_MIN_DEG;
    private static final double TURRET_MAX_DEG = V3AutoConfig.TURRET_MAX_DEG;
    private static final double TURRET_OFFSET_DEG = V3AutoConfig.TURRET_OFFSET_DEG;

    // Extra intake move after gate intake
    private static final double EXTRA_GATE_INTAKE_Y_IN = V3AutoConfig.EXTRA_GATE_INTAKE_Y_IN;

    // Dynamic shot control toggles
    private boolean enableDynamicShotControl = false;
    private boolean enableShotOnMoveComp = false;

    // Predicted distance smoothing
    private static final double PREDICTED_DISTANCE_ALPHA = V3AutoConfig.PREDICTED_DISTANCE_ALPHA;

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
        isRedAlliance = AllianceStore.isRed(hardwareMap.appContext);
        AutoStartStore.setClose(hardwareMap.appContext);

        Pose blueStartPose = new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEG)
        );
        startPose = AllianceMirror.mirrorPose(blueStartPose, isRedAlliance);

        intake = new Intake(hardwareMap);

        hood = new Hood(hardwareMap);
        feeder = new Feeder(hardwareMap);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new Flywheel(hardwareMap, battery);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.updatePose();
        follower.setMaxPower(1.0);

        turret = new Turret(hardwareMap, RobotConfig.TURRET_MOTOR, DcMotorSimple.Direction.REVERSE);

        double blueFirstShotX = START_X + V3AutoConfig.FIRST_SHOT_FWD_IN * Math.cos(Math.toRadians(START_HEADING_DEG));
        double blueFirstShotY = START_Y + V3AutoConfig.FIRST_SHOT_SIDE_IN * Math.sin(Math.toRadians(START_HEADING_DEG));
        firstShotPose = AllianceMirror.mirrorPose(
                new Pose(blueFirstShotX, blueFirstShotY, 0),
                isRedAlliance
        );

        buildPaths();

        intake.setPower(0.0);
        flywheel.stop();
        feeder.clutchIn();
        feeder.armBlock();
        hoodAngleDeg = hood.setAngle(FIRST_SHOT_HOOD_DEG);
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

            turret.setAngle(V3AutoConfig.INIT_TURRET_ANGLE_DEG);
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
        hoodAngleDeg = hood.setAngle(hoodAngleDeg);
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
                    hoodAngleDeg = hood.setAngle(FIRST_SHOT_HOOD_DEG);
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
        intake.setPower(0.0);
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

        toLine2 = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(V3AutoConfig.LINE2_C1X, V3AutoConfig.LINE2_C1Y),
                        p(V3AutoConfig.LINE2_C2X, V3AutoConfig.LINE2_C2Y),
                        p(V3AutoConfig.LINE2_ENDX, V3AutoConfig.LINE2_ENDY)
                )
        );
        toLine2.setConstantHeadingInterpolation(h(V3AutoConfig.LINE_HEADING_DEG));

        backToShoot = new Path(
                new BezierCurve(
                        p(V3AutoConfig.BTS_STARTX, V3AutoConfig.BTS_STARTY),
                        p(V3AutoConfig.BTS_C1X, V3AutoConfig.BTS_C1Y),
                        firstShotPose
                )
        );
        backToShoot.reverseHeadingInterpolation();

        toGateOpenGate = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(V3AutoConfig.GATE_C1X, V3AutoConfig.GATE_C1Y),
                        p(V3AutoConfig.GATE_ENDX, V3AutoConfig.GATE_ENDY)
                )
        );
        toGateOpenGate.setLinearHeadingInterpolation(
                h(V3AutoConfig.GATE_OPEN_HEADING_START_DEG),
                h(V3AutoConfig.GATE_OPEN_HEADING_END_DEG),
                V3AutoConfig.GATE_OPEN_HEADING_T);

        toGateIntake = new Path(
                new BezierLine(
                        p(V3AutoConfig.GI_STARTX, V3AutoConfig.GI_STARTY),
                        p(V3AutoConfig.GI_ENDX, V3AutoConfig.GI_ENDY)
                )
        );
        toGateIntake.setConstantHeadingInterpolation(h(V3AutoConfig.GATE_INTAKE_HEADING_DEG));

        toFourthPickup = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(V3AutoConfig.FP_C1X, V3AutoConfig.FP_C1Y),
                        p(V3AutoConfig.FP_ENDX, V3AutoConfig.FP_ENDY)
                )
        );
        toFourthPickup.setConstantHeadingInterpolation(h(V3AutoConfig.LINE_HEADING_DEG));

        backToFinalShoot = new Path(
                new BezierLine(
                        p(V3AutoConfig.FP_ENDX, V3AutoConfig.FP_ENDY),
                        firstShotPose
                )
        );
        backToFinalShoot.reverseHeadingInterpolation();

        toLastLine = new Path(
                new BezierCurve(
                        firstShotPose,
                        p(V3AutoConfig.LL_C1X, V3AutoConfig.LL_C1Y),
                        p(V3AutoConfig.LL_C2X, V3AutoConfig.LL_C2Y),
                        p(V3AutoConfig.LL_ENDX, V3AutoConfig.LL_ENDY)
                )
        );
        toLastLine.setTangentHeadingInterpolation();

        Pose lastReturnEndPose = AllianceMirror.mirrorPose(
                new Pose(
                        (START_X + V3AutoConfig.FIRST_SHOT_FWD_IN * Math.cos(Math.toRadians(START_HEADING_DEG))) + V3AutoConfig.LAST_RETURN_END_DX,
                        (START_Y + V3AutoConfig.FIRST_SHOT_SIDE_IN * Math.sin(Math.toRadians(START_HEADING_DEG))) + V3AutoConfig.LAST_RETURN_END_DY,
                        0
                ),
                isRedAlliance
        );

        backToShootFromLastLine = new Path(
                new BezierCurve(
                        p(V3AutoConfig.BFLL_STARTX, V3AutoConfig.BFLL_STARTY),
                        p(V3AutoConfig.BFLL_C1X, V3AutoConfig.BFLL_C1Y),
                        lastReturnEndPose
                )
        );
        backToShootFromLastLine.reverseHeadingInterpolation();
    }

    private void updateAutoState() {
        switch (autoState) {
            case DRIVE_TO_FIRST_SHOT:
                if (feedState == FeedState.IDLE && follower.getCurrentTValue() > V3AutoConfig.FIRST_SHOT_FEED_TVALUE) {
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
                if (follower.getCurrentTValue() > V3AutoConfig.LINE2_SLOW_TVALUE) {
                    follower.setMaxPower(V3AutoConfig.LINE2_SLOW_POWER);
                    toLine2.setConstantHeadingInterpolation(h(V3AutoConfig.LINE_HEADING_DEG));
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
                    feeder.armBlock();
                    feeder.clutchOut();
                    startThirdCycleToGate();
                    intake.setPower(0);
                }
                break;

            case DRIVE_TO_GATE:
                intake.setPower(0);
                if (!follower.isBusy()) {
                    autoTimer.reset();
                    autoState = AutoState.WAIT_AT_GATE;
                }
                break;

            case WAIT_AT_GATE:
                intake.setPower(0.0);   // keep intake off while tapping/opening gate
                if (autoTimer.seconds() >= V3AutoConfig.WAIT_AT_GATE_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, false);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE;
                }
                break;

            case WAIT_FOR_GATE_INTAKE:
                intake.setPower(follower.getCurrentTValue() >= V3AutoConfig.GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= V3AutoConfig.GATE_INTAKE_SEC) {
                    startExtraGateIntakeMove();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE:
                intake.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= V3AutoConfig.EXTRA_MOVE_TIMEOUT_SEC) {
                    startReturnFromGateToShoot();
                }
                break;

            case DRIVE_BACK_TO_SHOOT_THIRD:
                intake.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_THIRD;
                }
                break;

            case SHOOT_THIRD:
                if (feedState == FeedState.DONE) {
                    feeder.armBlock();
                    feeder.clutchOut();
                    startFourthPickupCycle();
                }
                break;

            case DRIVE_TO_FOURTH_PICKUP:
                if (!follower.isBusy()) {
                    startReturnToFinalShoot();
                }
                break;

            case DRIVE_BACK_TO_FINAL_SHOOT:
                intake.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_FOURTH;
                }
                break;

            case SHOOT_FOURTH:
                if (feedState == FeedState.DONE) {
                    feeder.armBlock();
                    feeder.clutchOut();
                    startGateCycleAgain();
                    intake.setPower(0);
                }
                break;

            case DRIVE_TO_GATE_AGAIN:
                intake.setPower(0);
                if (!follower.isBusy()) {
                    autoTimer.reset();
                    autoState = AutoState.WAIT_AT_GATE_AGAIN;
                }
                break;

            case WAIT_AT_GATE_AGAIN:
                intake.setPower(0.0);
                if (autoTimer.seconds() >= V3AutoConfig.WAIT_AT_GATE_AGAIN_SEC) {
                    follower.setMaxPower(1.0);
                    follower.followPath(toGateIntake, true);
                    autoTimer.reset();
                    autoState = AutoState.WAIT_FOR_GATE_INTAKE_AGAIN;
                }
                break;

            case WAIT_FOR_GATE_INTAKE_AGAIN:
                intake.setPower(follower.getCurrentTValue() >= V3AutoConfig.GATE_INTAKE_ON_TVALUE ? 1.0 : 0.0);
                if (autoTimer.seconds() >= V3AutoConfig.GATE_INTAKE_AGAIN_SEC) {
                    startExtraGateIntakeMoveAgain();
                }
                break;

            case DRIVE_EXTRA_INTAKE_AT_GATE_AGAIN:
                intake.setPower(1.0);
                if (!follower.isBusy() || autoTimer.seconds() >= V3AutoConfig.EXTRA_MOVE_TIMEOUT_SEC) {
                    startReturnFromGateToShootAgain();
                }
                break;

            case DRIVE_BACK_TO_SHOOT_FIFTH:
                intake.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_FIFTH;
                }
                break;

            case SHOOT_FIFTH:
                if (feedState == FeedState.DONE) {
                    feeder.armBlock();
                    feeder.clutchOut();
                    startLastLineCycle();
                }
                break;

            case DRIVE_TO_LAST_LINE:
                if (follower.getCurrentTValue() > V3AutoConfig.LAST_LINE_SLOW_TVALUE) {
                    follower.setMaxPower(V3AutoConfig.LAST_LINE_SLOW_POWER);
                    toLastLine.setConstantHeadingInterpolation(h(V3AutoConfig.LINE_HEADING_DEG));
                }
                if (!follower.isBusy()) {
                    intake.setPower(1.0);
                    follower.setMaxPower(1.0);
                    follower.followPath(backToShootFromLastLine, true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT_LAST;

                    enableDynamicShotControl = true;
                    enableShotOnMoveComp = true;
                }
                break;

            case DRIVE_BACK_TO_SHOOT_LAST:
                intake.setPower(1.0);
                if (follower.getCurrentTValue() > V3AutoConfig.BACK_TO_SHOOT_LAST_SWITCH_TVALUE) {
                    backToShootFromLastLine.setConstantHeadingInterpolation(Math.toRadians(V3AutoConfig.BACK_TO_SHOOT_LAST_HEADING_DEG));
                }
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_LAST;
                }
                break;

            case SHOOT_LAST:
                if (feedState == FeedState.DONE) {
                    intake.setPower(0.0);
                    feeder.armBlock();
                    feeder.clutchOut();
                    enableDynamicShotControl = false;
                    enableShotOnMoveComp = false;
                    turret.setAngle(0);
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                turret.setAngle(0);
                intake.setPower(0.0);
                break;
        }
    }

    private void finishFirstShotAndStartLine2() {
        feeder.clutchOut();
        feeder.armBlock();

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toLine2, false);
        autoState = AutoState.DRIVE_TO_LINE2;
    }

    private void startThirdCycleToGate() {
        intake.setPower(1.0);
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
        gateExtraIntakeMove.setConstantHeadingInterpolation(h(V3AutoConfig.GATE_INTAKE_HEADING_DEG));

        follower.followPath(gateExtraIntakeMove, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE;
    }

    private void startReturnFromGateToShoot() {
        follower.setMaxPower(1.0);

        backToShootFromGate = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        p(V3AutoConfig.RG1_C1X, V3AutoConfig.RG1_C1Y),
                        firstShotPose
                )
        );
        backToShootFromGate.reverseHeadingInterpolation();

        intake.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.followPath(backToShootFromGate, true);
        autoState = AutoState.DRIVE_BACK_TO_SHOOT_THIRD;
    }

    private void startFourthPickupCycle() {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toFourthPickup, false);
        autoState = AutoState.DRIVE_TO_FOURTH_PICKUP;
    }

    private void startReturnToFinalShoot() {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(backToFinalShoot, false);
        autoState = AutoState.DRIVE_BACK_TO_FINAL_SHOOT;
    }

    private void startGateCycleAgain() {
        intake.setPower(1.0);
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
        gateExtraIntakeMoveAgain.setConstantHeadingInterpolation(h(V3AutoConfig.GATE_INTAKE_HEADING_DEG));

        follower.followPath(gateExtraIntakeMoveAgain, true);
        autoState = AutoState.DRIVE_EXTRA_INTAKE_AT_GATE_AGAIN;
    }

    private void startReturnFromGateToShootAgain() {
        follower.setMaxPower(1.0);

        backToShootFromGateAgain = new Path(
                new BezierCurve(
                        new Pose(follower.getPose().getX(), follower.getPose().getY()),
                        p(V3AutoConfig.RG2_C1X, V3AutoConfig.RG2_C1Y),
                        firstShotPose
                )
        );
        backToShootFromGateAgain.reverseHeadingInterpolation();

        intake.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.followPath(backToShootFromGateAgain, true);
        autoState = AutoState.DRIVE_BACK_TO_SHOOT_FIFTH;
    }

    private void startLastLineCycle() {
        intake.setPower(1.0);
        feedState = FeedState.IDLE;

        enableDynamicShotControl = true;
        enableShotOnMoveComp = true;

        follower.setMaxPower(1.0);
        follower.followPath(toLastLine, false);
        autoState = AutoState.DRIVE_TO_LAST_LINE;
    }

    private void startFeedSequence() {
        feeder.armShoot();
        feedTimer.reset();
        feedState = FeedState.WAIT_BEFORE_INTAKE;
    }

    private void updateFeedSequence() {

        if (reversingIntake) {
            if (reverseTimer.seconds() < V3AutoConfig.REVERSE_TIME_SEC) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(1.0);
                reversingIntake = false;
            }
        }

        switch (feedState) {
            case IDLE:
                break;

            case WAIT_BEFORE_INTAKE:
                feeder.clutchIn();
                intake.setPower(0.0);

                if (feedTimer.seconds() >= V3AutoConfig.FEED_START_DELAY_SEC) {
                    intake.setPower(1.0);
                    feedState = FeedState.RUN_INTAKE;
                }
                break;

            case RUN_INTAKE:
                if (feedTimer.seconds() >= V3AutoConfig.FEED_TOTAL_TIME_SEC) {
                    reversingIntake = true;
                    reverseTimer.reset();

                    intake.setPower(0.0);
                    feedState = FeedState.DONE;
                    feeder.clutchOut();
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
        return V3AutoConfig.SHOT_TIME_SEC;
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
        double[][] shotTable = V3AutoConfig.SHOT_TABLE;

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
                targetVelocityRad = (v1 + t * (v2 - v1)) + 15;
                hoodAngleDeg = hood.setAngle(hoodAngleDeg);
                return;
            }
        }
    }
}