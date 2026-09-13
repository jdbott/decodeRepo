/*
 * ============================================================
 * Decode Near Auto
 * ============================================================
 * STARTING POSE:    (33.11118657953564, 125.85974018244295)
 * STARTING HEADING: 180 degrees (Math.toRadians(90))
 * ============================================================
 * SHOOTING COORDINATES (5 stops):
 *   1. (33.0, 141.0)
 *   2. (48.0, 99.4)
 *   3. (54.0, 93.0)
 *   4. (53.0, 90.0)
 *   5. (63.0, 82.0)
 *
 * INTAKE ZONES:
 *   1. Through (35,85) -> (18,82)
 *   2. Through (33,60) -> (18,59)
 *   3. At (8,67)  - 3.0 second dwell
 *   4. At (8,67)  - 3.0 second dwell (second visit)
 *
 * TIMING REQUIREMENTS:
 *   - Flywheel spin-up: 1.5 s before each shot
 *   - Shot dwell:       1.5 s at each shooting coordinate
 *   - Intake pre-start: 2.0 s before intake zones (via throttled path speed)
 *   - (8,67) dwell:     3.0 s each visit
 * ============================================================
 */

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
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.autoshared.V3ClosePartnerConfig;
import org.firstinspires.ftc.teamcode.hardwareClasses.Flywheel;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Near Auto: Yuvi", group = "Autonomous")
public class NearAutoByYuvi extends LinearOpMode {

    // ===== Hardware =====
    private Follower follower;
    private Flywheel flywheel;
    private DcMotorEx intakeMotor;
    private Turret turret;
    private Servo hoodServo;
    private Servo armServo;
    private Servo clutchServo;

    // ===== Alliance =====
    private boolean isRedAlliance = false;

    // ===== Timers =====
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();

    // ===== Auto State Machine =====
    private enum AutoState {
        INIT,

        // Shot 1 at (33, 141)
        DRIVE_TO_SHOT_1,
        PREPARE_SHOT_1,
        SHOOT_1,

        // Drive to (48, 99.4)
        DRIVE_TO_SHOT_2,
        PREPARE_SHOT_2,
        SHOOT_2,

        // Intake through (35,85) -> (18,82)
        DRIVE_TO_INTAKE_1,
        INTAKE_1,

        // Drive to (54, 93)
        DRIVE_TO_SHOT_3,
        PREPARE_SHOT_3,
        SHOOT_3,

        // Intake through (33,60) -> (18,59)
        DRIVE_TO_INTAKE_2,
        INTAKE_2,

        // Drive to (53, 90)
        DRIVE_TO_SHOT_4,
        PREPARE_SHOT_4,
        SHOOT_4,

        // Intake at (8, 67) - 3 s dwell
        DRIVE_TO_INTAKE_3,
        WAIT_AT_INTAKE_3,

        // Drive to (63, 82)
        DRIVE_TO_SHOT_5,
        PREPARE_SHOT_5,
        SHOOT_5,

        // Intake at (8, 67) again - 3 s dwell
        DRIVE_TO_INTAKE_4,
        WAIT_AT_INTAKE_4,

        // Park
        DRIVE_TO_PARK,
        DONE
    }
    private AutoState autoState = AutoState.INIT;

    // ===== Feed / Shooting Sequence =====
    private enum FeedState {
        IDLE,
        WAIT_BEFORE_FEED,
        FEEDING,
        DONE
    }
    private FeedState feedState = FeedState.IDLE;

    // ===== Timing Constants =====
    private static final double FLYWHEEL_SPIN_UP_SEC = 1.2;
    private static final double SHOT_DWELL_SEC       = 1;
    private static final double INTAKE_3_DWELL_SEC   = 2.25;
    private static final double INTAKE_4_DWELL_SEC   = 2.25;
    private static final double FEED_START_DELAY_SEC = 0.08;
    private static final double FEED_TOTAL_SEC       = 0.75;

    // ===== Shot Control =====
    private boolean enableDynamicShotControl = true;
    private double hoodAngleDeg = 48.0;
    private double targetVelocityRad = 0.0;

    // ===== Paths =====
    private Path toShot1;
    private Path toShot2;
    private Path toIntake1;
    private Path toShot3;
    private Path toIntake2;
    private Path toShot4;
    private Path toIntake3;
    private Path toShot5;
    private Path toIntake4;
    private Path toPark;

    // ===== Poses (Blue Alliance, mirrored if Red) =====
    private Pose startPose;
    private Pose shot1Pose;
    private Pose shot2Pose;
    private Pose intake1Control;
    private Pose intake1End;
    private Pose shot3Pose;
    private Pose intake2Control;
    private Pose intake2End;
    private Pose shot4Pose;
    private Pose intake3Pose;
    private Pose shot5Pose;
    private Pose parkPose;

    // -------------------------------------------------------------------------
    // MAIN LOOP
    // -------------------------------------------------------------------------
    @Override
    public void runOpMode() {
        isRedAlliance = AllianceStore.isRed(hardwareMap.appContext);

        initPoses();
        initHardware();
        buildPaths();
        initMechanisms();

        telemetry.addLine("Decode Near Auto Initialized");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Start", startPose);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Begin state machine
        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_SHOT_1;
        prepareShot(V3ClosePartnerConfig.FIRST_SHOT_HOOD_DEG,
                V3ClosePartnerConfig.FIRST_SHOT_FLYWHEEL_RAD);

        follower.followPath(toShot1, false);

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();

            // Turret tracks goal during all non-intake, non-park states
            if (enableDynamicShotControl) {
                trackGoalFromOdometry(pose, false);
            } else {
                turret.setAngle(0);
            }

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();
            turret.update();

            updateFeedSequence();
            updateAutoState();

            telemetry.addData("State", autoState);
            telemetry.addData("Feed", feedState);
            telemetry.addData("Pose", pose);
            telemetry.addData("Flywheel RPM", Math.toDegrees(flywheel.getVelocityRadPerSec()) / 6.0);
            telemetry.addData("Turret", turret.getCurrentAngle());
            telemetry.update();
        }

        stopAll();
    }

    // -------------------------------------------------------------------------
    // STATE MACHINE
    // -------------------------------------------------------------------------
    private void updateAutoState() {
        switch (autoState) {

            // ---------------------------------------------------------
            // SHOT 1: (33, 141)
            // ---------------------------------------------------------
            // ---------------------------------------------------------
// SHOT 1
// ---------------------------------------------------------
            case DRIVE_TO_SHOT_1:
                intakeMotor.setPower(0.0);
                if (!follower.isBusy()) {
                    stateTimer.reset(); // Reset to accurately track spin-up at the spot
                    autoState = AutoState.PREPARE_SHOT_1;
                }
                break;

            case PREPARE_SHOT_1:
                intakeMotor.setPower(0.5);
                if (stateTimer.seconds() >= FLYWHEEL_SPIN_UP_SEC) {
                    startFeedSequence();
                    stateTimer.reset();
                    autoState = AutoState.SHOOT_1;
                }
                break;

            case SHOOT_1:
                intakeMotor.setPower(0.5);
                if (feedState == FeedState.DONE) {
                    endShotAndDriveToShot2();
                }
                break;

// ---------------------------------------------------------
// SHOT 2
// ---------------------------------------------------------
            case DRIVE_TO_SHOT_2:
                intakeMotor.setPower(0.0);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.PREPARE_SHOT_2;
                }
                break;

            case PREPARE_SHOT_2:
                intakeMotor.setPower(0.5);
                if (stateTimer.seconds() >= FLYWHEEL_SPIN_UP_SEC) {
                    startFeedSequence();
                    stateTimer.reset();
                    autoState = AutoState.SHOOT_2;
                }
                break;

            case SHOOT_2:
                intakeMotor.setPower(0.5);
                if (feedState == FeedState.DONE) {
                    endShotAndDriveToIntake1(); // Advanced to Intake 1
                }
                break;

// ---------------------------------------------------------
// INTAKE 1
// ---------------------------------------------------------
            case DRIVE_TO_INTAKE_1:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.INTAKE_1;
                }
                break;

            case INTAKE_1:
                intakeMotor.setPower(1.0);
                if (stateTimer.seconds() >= 0.4) {
                    intakeMotor.setPower(0.0);
                    driveToShot3();
                }
                break;

// ---------------------------------------------------------
// SHOT 3
// ---------------------------------------------------------
            case DRIVE_TO_SHOT_3:
                intakeMotor.setPower(0.0);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.PREPARE_SHOT_3;
                }
                break;

            case PREPARE_SHOT_3:
                intakeMotor.setPower(0.5);
                if (stateTimer.seconds() >= FLYWHEEL_SPIN_UP_SEC) {
                    startFeedSequence();
                    stateTimer.reset();
                    autoState = AutoState.SHOOT_3;
                }
                break;

            case SHOOT_3:
                intakeMotor.setPower(0.5);
                if (feedState == FeedState.DONE) {
                    endShotAndDriveToIntake2(); // Advanced to Intake 2
                }
                break;

// ---------------------------------------------------------
// INTAKE 2
// ---------------------------------------------------------
            case DRIVE_TO_INTAKE_2:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.INTAKE_2;
                }
                break;

            case INTAKE_2:
                intakeMotor.setPower(1.0);
                if (stateTimer.seconds() >= 0.4) {
                    intakeMotor.setPower(0.0);
                    driveToShot4();
                }
                break;

// ---------------------------------------------------------
// SHOT 4
// ---------------------------------------------------------
            case DRIVE_TO_SHOT_4:
                intakeMotor.setPower(0.0);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.PREPARE_SHOT_4;
                }
                break;

            case PREPARE_SHOT_4:
                intakeMotor.setPower(0.5);
                if (stateTimer.seconds() >= FLYWHEEL_SPIN_UP_SEC) {
                    startFeedSequence();
                    stateTimer.reset();
                    autoState = AutoState.SHOOT_4;
                }
                break;

            case SHOOT_4:
                intakeMotor.setPower(0.5);
                if (feedState == FeedState.DONE) {
                    endShotAndDriveToIntake3(); // Dropped the unnecessary standalone SHOT_DWELL_SEC
                }
                break;

// ---------------------------------------------------------
// INTAKE 3
// ---------------------------------------------------------
            case DRIVE_TO_INTAKE_3:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.WAIT_AT_INTAKE_3;
                }
                break;

            case WAIT_AT_INTAKE_3:
                intakeMotor.setPower(1.0);
                if (stateTimer.seconds() >= INTAKE_3_DWELL_SEC) {
                    intakeMotor.setPower(0.0);
                    driveToShot5();
                }
                break;

// ---------------------------------------------------------
// SHOT 5
// ---------------------------------------------------------
            case DRIVE_TO_SHOT_5:
                intakeMotor.setPower(0.0);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.PREPARE_SHOT_5;
                }
                break;

            case PREPARE_SHOT_5:
                intakeMotor.setPower(0.5);
                if (stateTimer.seconds() >= FLYWHEEL_SPIN_UP_SEC) {
                    startFeedSequence();
                    stateTimer.reset();
                    autoState = AutoState.SHOOT_5;
                }
                break;

            case SHOOT_5:
                intakeMotor.setPower(0.5);
                if (feedState == FeedState.DONE) {
                    endShotAndDriveToIntake4(); // Advanced to final Intake 4 visit
                }
                break;

            // ---------------------------------------------------------
            // INTAKE 4: at (8, 67) again - 3 s dwell
            // ---------------------------------------------------------
            case DRIVE_TO_INTAKE_4:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.WAIT_AT_INTAKE_4;
                }
                break;

            case WAIT_AT_INTAKE_4:
                intakeMotor.setPower(1.0);
                if (stateTimer.seconds() >= INTAKE_4_DWELL_SEC) {
                    intakeMotor.setPower(0.0);
                    driveToPark();
                }
                break;

            // ---------------------------------------------------------
            // PARK
            // ---------------------------------------------------------
            case DRIVE_TO_PARK:
                intakeMotor.setPower(0.0);
                if (!follower.isBusy()) {
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                stopAll();
                break;
        }
    }

    // -------------------------------------------------------------------------
    // TRANSITION HELPERS
    // -------------------------------------------------------------------------
    private void endShotAndDriveToShot2() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = true;

        follower.setMaxPower(1.0);
        follower.followPath(toShot2, false);

        prepareShot(V3ClosePartnerConfig.FIRST_SHOT_HOOD_DEG,
                V3ClosePartnerConfig.FIRST_SHOT_FLYWHEEL_RAD);
        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_SHOT_2;
    }

    private void endShotAndDriveToIntake1() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = false;

        // Throttle speed so path takes >2 s; intake starts now
        follower.setMaxPower(1.0);
        follower.followPath(toIntake1, false);
        intakeMotor.setPower(1.0);

        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_INTAKE_1;
    }

    private void driveToShot3() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = true;

        follower.setMaxPower(1.0);
        follower.followPath(toShot3, false);

        prepareShot(V3ClosePartnerConfig.FIRST_SHOT_HOOD_DEG,
                V3ClosePartnerConfig.FIRST_SHOT_FLYWHEEL_RAD);
        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_SHOT_3;
    }

    private void endShotAndDriveToIntake2() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = false;

        follower.setMaxPower(1.0);
        follower.followPath(toIntake2, false);
        intakeMotor.setPower(1.0);

        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_INTAKE_2;
    }

    private void driveToShot4() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = true;

        follower.setMaxPower(1.0);
        follower.followPath(toShot4, false);

        prepareShot(V3ClosePartnerConfig.FIRST_SHOT_HOOD_DEG,
                V3ClosePartnerConfig.FIRST_SHOT_FLYWHEEL_RAD);
        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_SHOT_4;
    }

    private void endShotAndDriveToIntake3() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = false;

        follower.setMaxPower(0.6);
        follower.followPath(toIntake3, false);
        intakeMotor.setPower(1.0);

        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_INTAKE_3;
    }

    private void driveToShot5() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = true;

        follower.setMaxPower(1.0);
        follower.followPath(toShot5, false);

        prepareShot(V3ClosePartnerConfig.FIRST_SHOT_HOOD_DEG,
                V3ClosePartnerConfig.FIRST_SHOT_FLYWHEEL_RAD);
        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_SHOT_5;
    }

    private void endShotAndDriveToIntake4() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = false;

        follower.setMaxPower(0.6);
        follower.followPath(toIntake4, false);
        intakeMotor.setPower(1.0);

        stateTimer.reset();
        autoState = AutoState.DRIVE_TO_INTAKE_4;
    }

    private void driveToPark() {
        clutchOut();
        armBlock();
        enableDynamicShotControl = false;
        flywheel.stop();
        turret.setAngle(0);

        follower.setMaxPower(1.0);
        follower.followPath(toPark, false);
        autoState = AutoState.DRIVE_TO_PARK;
    }

    // -------------------------------------------------------------------------
    // SHOT / FEED LOGIC
    // -------------------------------------------------------------------------
    private void prepareShot(double hoodDeg, double flywheelRad) {
        hoodAngleDeg = hoodDeg;
        targetVelocityRad = flywheelRad;
        setHoodAngle(hoodAngleDeg);
        flywheel.setTargetVelocity(targetVelocityRad);
    }

    private void startFeedSequence() {
        armShoot();
        feedTimer.reset();
        feedState = FeedState.WAIT_BEFORE_FEED;
    }

    private void updateFeedSequence() {
        switch (feedState) {
            case IDLE:
                break;

            case WAIT_BEFORE_FEED:
                clutchIn();
                intakeMotor.setPower(0.0);
                if (feedTimer.seconds() >= FEED_START_DELAY_SEC) {
                    intakeMotor.setPower(1.0);
                    feedState = FeedState.FEEDING;
                }
                break;

            case FEEDING:
                if (feedTimer.seconds() >= FEED_TOTAL_SEC) {
                    intakeMotor.setPower(0.0);
                    feedState = FeedState.DONE;
                    clutchOut();
                }
                break;

            case DONE:
                break;
        }
    }

    // -------------------------------------------------------------------------
    // TURRET TRACKING (from V3ClosePartner)
    // -------------------------------------------------------------------------
    private void trackGoalFromOdometry(Pose pose, boolean useShotOnMoveComp) {
        double targetX = getTargetX();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double fieldVx = follower.getVelocity().getXComponent();
        double fieldVy = follower.getVelocity().getYComponent();

        double turretX = robotX - V3ClosePartnerConfig.TURRET_CENTER_OFFSET_IN * Math.cos(robotHeadingRad);
        double turretY = robotY - V3ClosePartnerConfig.TURRET_CENTER_OFFSET_IN * Math.sin(robotHeadingRad);

        double compensatedTargetX = targetX;
        double compensatedTargetY = V3ClosePartnerConfig.TARGET_Y;

        if (useShotOnMoveComp) {
            double shotTime = V3ClosePartnerConfig.SHOT_TIME_SEC;
            compensatedTargetX = targetX - fieldVx * shotTime;
            compensatedTargetY = V3ClosePartnerConfig.TARGET_Y - fieldVy * shotTime;
        }

        double dx = compensatedTargetX - turretX;
        double dy = compensatedTargetY - turretY;

        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double angleToTargetRobotDeg = normalize180(angleToTargetFieldDeg - robotHeadingDeg);
        double desiredTurretDeg = normalize180(angleToTargetRobotDeg + V3ClosePartnerConfig.TURRET_OFFSET_DEG);

        double safeTurretDeg = wrapIntoTurretWindow(
                desiredTurretDeg,
                turret.getCurrentAngle(),
                V3ClosePartnerConfig.TURRET_MIN_DEG,
                V3ClosePartnerConfig.TURRET_MAX_DEG
        );

        turret.setAngle(safeTurretDeg);
    }

    private double getTargetX() {
        return AllianceMirror.mirrorX(V3ClosePartnerConfig.BLUE_TARGET_X, isRedAlliance);
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

    // -------------------------------------------------------------------------
    // POSE / PATH / HARDWARE INIT
    // -------------------------------------------------------------------------
    private void initPoses() {
        startPose       = p(33.11118657953564, 125.85974018244295, 180.0);
        shot1Pose       = p(43.0, 110.0, 90.0);
        shot2Pose       = p(48.0, 99.4, 90.0);
        intake1Control  = p(45.0, 83.0, 90.0);
        intake1End      = p(18.3, 79.2, 90.0);
        shot3Pose       = p(54.0, 93.0, 90.0);
        intake2Control  = p(33.0, 60.0, 90.0);
        intake2End      = p(18.0, 54.0, 90.0);
        shot4Pose       = p(45.0, 75.0, 90.0);
        intake3Pose     = p(8.0, 67.0, 90.0);
        shot5Pose       = p(63.0, 82.0, 90.0);
        parkPose        = p(56.0, 129.0, 270.0);
    }

    private Pose p(double x, double y, double headingDeg) {
        return AllianceMirror.mirrorPose(
                new Pose(x, y, Math.toRadians(headingDeg)),
                isRedAlliance
        );
    }

    private void initHardware() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo   = hardwareMap.get(Servo.class, RobotConfig.HOOD_SERVO);
        armServo    = hardwareMap.get(Servo.class, RobotConfig.FEEDER_ARM_SERVO);
        clutchServo = hardwareMap.get(Servo.class, RobotConfig.FEEDER_CLUTCH_SERVO);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new Flywheel(hardwareMap, battery);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.updatePose();
        follower.setMaxPower(1.0);

        turret = new Turret(hardwareMap, RobotConfig.TURRET_MOTOR, DcMotorSimple.Direction.REVERSE);
    }

    private void buildPaths() {
        toShot1 = new Path(new BezierLine(startPose, shot1Pose));
        toShot1.setTangentHeadingInterpolation();

        toShot2 = new Path(new BezierLine(shot1Pose, shot2Pose));
        toShot2.setTangentHeadingInterpolation();

        toIntake1 = new Path(new BezierLine(shot2Pose, intake1End));
        toIntake1.setConstantHeadingInterpolation(Math.toRadians(90));

        toShot3 = new Path(new BezierLine(intake1End, shot3Pose));
        toShot3.setConstantHeadingInterpolation(Math.toRadians(90));

        toIntake2 = new Path(new BezierLine(shot3Pose, intake2End));
        toIntake2.setConstantHeadingInterpolation(Math.toRadians(90));

        toShot4 = new Path(new BezierLine(intake2End, shot4Pose));
        toShot4.setTangentHeadingInterpolation();

        toIntake3 = new Path(new BezierLine(shot4Pose, intake3Pose));
        toIntake3.setTangentHeadingInterpolation();

        toShot5 = new Path(new BezierLine(intake3Pose, shot5Pose));
        toShot5.setTangentHeadingInterpolation();

        toIntake4 = new Path(new BezierLine(shot5Pose, intake3Pose));
        toIntake4.setTangentHeadingInterpolation();

        toPark = new Path(new BezierLine(intake3Pose, parkPose));
        toPark.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270));
    }

    private void initMechanisms() {
        intakeMotor.setPower(0.0);
        flywheel.stop();
        clutchIn();
        armBlock();
        setHoodAngle(V3ClosePartnerConfig.FIRST_SHOT_HOOD_DEG);
        turret.setAngle(90.0);
    }

    // -------------------------------------------------------------------------
    // SERVO / MECHANISM HELPERS
    // -------------------------------------------------------------------------
    private void clutchIn()  { clutchServo.setPosition(0.48); }
    private void clutchOut() { clutchServo.setPosition(0.52); }
    private void armBlock()  { armServo.setPosition(0.28); }
    private void armShoot()  { armServo.setPosition(0.42); }

    private void setHoodAngle(double angleDeg) {
        final double MIN_ANGLE = 30.0;
        final double MAX_ANGLE = 60.0;
        final double MIN_POS   = 0.42;
        final double MAX_POS   = 0.95;

        double clamped = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDeg));
        double t = (clamped - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        double pos = MIN_POS + t * (MAX_POS - MIN_POS);
        hoodServo.setPosition(Math.max(MIN_POS, Math.min(MAX_POS, pos)));
    }

    private void stopAll() {
        flywheel.stop();
        intakeMotor.setPower(0.0);
        turret.setAngle(0);
        clutchOut();
        armBlock();
    }
}