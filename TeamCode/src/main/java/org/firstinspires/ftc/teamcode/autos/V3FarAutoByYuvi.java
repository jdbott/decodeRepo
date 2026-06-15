package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.RobotConfig;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name = "V3 Far Auto: Yuvi Edition")
public class V3FarAutoByYuvi extends LinearOpMode {

    private Hood hood;
    private Feeder feeder;

    private Follower follower;
    private Turret turret;
    private Intake intake;
    private Flywheel flywheel;

    private boolean isRedAlliance = false;

    // ===== Fixed shot settings =====
    private static final double FIXED_HOOD_DEG = 53.5;
    private static final double FIXED_FLYWHEEL_RAD = 445;

    // ===== Field positions (BLUE-NATIVE) =====
    private static final double START_X = 56.0;
    private static final double START_Y = 8.0;
    private static final double START_HEADING_DEG = 90.0;

    private static final double BLUE_TARGET_X = 5.0;
    private static final double TARGET_Y = 139.0;

    // ===== Turret config =====
    private static final double TURRET_CENTER_OFFSET_IN = 1.5;
    private static final double TURRET_MIN_DEG = -180.0;
    private static final double TURRET_MAX_DEG = 180.0;
    private static final double TURRET_OFFSET_DEG = 180;

    // ===== Timing =====
    private static final double FIRST_SHOT_DELAY_SEC = 2.25;
    private static final double FEED_START_DELAY_SEC = 0.10;
    private static final double FEED_TOTAL_TIME_SEC = 0.93;   // 3 preloaded balls ≈ 0.5–0.75 s
    private static final double REVERSE_TIME_SEC = 0.25;

    private static final double FLYWHEEL_PREP_SEC = 1.25;     // spin up before each shot
    private static final double INTAKE_DURATION_SEC = 2.78;   // total intake runtime per cycle
    private static final double INTAKE_START_DIST = 30.0;     // inches before intake zone (~1 s)
    private static final double FLYWHEEL_PREP_DIST = 22.0;    // inches before shoot point (~0.75 s)

    // ===== Fixed turret aim commands (BLUE-NATIVE) =====
    private static final double INIT_TURRET_ANGLE_DEG = 113;

    // ===== Key poses =====
    private Pose startPose;
    private Pose intake1StartPose;
    private Pose intake1EndPose;
    private Pose shoot1Pose;
    private Pose intake2StartPose;
    private Pose intake2EndPose;
    private Pose shoot2Pose;
    private Pose intake3StartPose;
    private Pose intake3EndPose;
    private Pose shoot3Pose;
    private Pose endPose;

    // ===== Paths =====
    private PathChain toIntake1Zone;
    private Path toShoot1;
    private PathChain toIntake2Zone;
    private Path toShoot2;
    private PathChain toIntake3Zone;
    private Path toShoot3;
    private Path toEnd;

    // ===== Timers =====
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime reverseTimer = new ElapsedTime();
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime flywheelPrepTimer = new ElapsedTime();

    private boolean reversingIntake = false;
    private boolean intakeRunning = false;
    private boolean flywheelPrepped = false;

    // ===== Cycle tracking =====
    private int extraCycleCount = 0;

    private enum FeedState {
        IDLE,
        WAIT_BEFORE_INTAKE,
        RUN_INTAKE,
        DONE
    }

    private FeedState feedState = FeedState.IDLE;

    private enum AutoState {
        WAIT_INITIAL_DELAY,
        SHOOT_PRELOAD,

        DRIVE_TO_INTAKE1,
        INTAKE1,

        DRIVE_TO_SHOOT1,
        SHOOT1,

        DRIVE_TO_INTAKE2,
        INTAKE2,

        DRIVE_TO_SHOOT2,
        SHOOT2,

        DRIVE_TO_INTAKE3,
        INTAKE3,

        DRIVE_TO_SHOOT3,
        SHOOT3,

        DRIVE_TO_END,
        DONE
    }

    private AutoState autoState = AutoState.WAIT_INITIAL_DELAY;

    @Override
    public void runOpMode() {
        isRedAlliance = AllianceStore.isRed(hardwareMap.appContext);
        AutoStartStore.setFar(hardwareMap.appContext);

        // ----- Mirror-wrapped Poses -----
        startPose        = p(START_X, START_Y, START_HEADING_DEG);
        intake1StartPose = p(3.520, 16.562, 180.0);
        intake1EndPose   = p(2.343, 2.837, 180.0);
        shoot1Pose       = p(70.646, 20.581, 180.0);
        intake2StartPose = p(33.219, 35.214, 180.0);
        intake2EndPose   = p(18.290, 35.059, 180.0);
        shoot2Pose       = p(69.914, 75.762, 180.0);
        intake3StartPose = p(32.187, 59.918, 180.0);
        intake3EndPose   = p(16.285, 59.078, 180.0);
        shoot3Pose       = p(68.186, 78.109, 180.0);
        endPose          = p(68.963, 16.829, 180.0);

        // ----- Hardware -----
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

        buildPaths();

        // ----- Initial mechanism state -----
        intake.setPower(0.0);
        flywheel.stop();
        feeder.clutchIn();
        feeder.armBlock();
        hood.setAngle(FIXED_HOOD_DEG);

        telemetry.addLine("Yuvi Auto Fixed Initialized");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        // ----- Init loop -----
        while (opModeInInit()) {
            follower.updatePose();
            turret.setAngle(mirrorTurretCommand(INIT_TURRET_ANGLE_DEG));
            turret.update();
            trackGoalFromOdometry(follower.getPose());

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Turret Angle", turret.getCurrentAngle());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // ----- On start -----
        hood.setAngle(FIXED_HOOD_DEG);
        flywheel.setTargetVelocity(FIXED_FLYWHEEL_RAD);

        stateTimer.reset();
        autoState = AutoState.WAIT_INITIAL_DELAY;

        while (opModeIsActive()) {
            follower.update();

            // ----- Dynamic goal turret tracking -----
            if (autoState != AutoState.DONE) {
                trackGoalFromOdometry(follower.getPose());
            } else {
                turret.setAngle(0.0);
            }
            turret.update();

            // ----- Flywheel always at target -----
            flywheel.setTargetVelocity(FIXED_FLYWHEEL_RAD);
            flywheel.update();

            updateFeedSequence();
            updateAutoState();

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Auto State", autoState);
            telemetry.addData("Feed State", feedState);
            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Turret Angle", turret.getCurrentAngle());
            telemetry.addData("Flywheel Actual", flywheel.getVelocityRadPerSec());
            telemetry.addData("Intake Running", intakeRunning);
            telemetry.addData("Flywheel Prepped", flywheelPrepped);
            telemetry.update();
        }

        flywheel.stop();
        intake.setPower(0.0);
    }

    // -------------------------------------------------------------------------
    //  Path Building — fully mirrored using the p() and h() helpers
    // -------------------------------------------------------------------------
    private void buildPaths() {
        // Intake 1 chain
        toIntake1Zone = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p(13.876, 18.959, 180.0)))
                .setLinearHeadingInterpolation(h(START_HEADING_DEG), h(180.0))
                .addPath(new BezierLine(p(13.876, 18.959, 180.0), intake1StartPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(intake1StartPose, intake1EndPose))
                .setTangentHeadingInterpolation()
                .build();

        // Shoot 1 path
        toShoot1 = new Path(new BezierLine(intake1EndPose, shoot1Pose));
        toShoot1.setTangentHeadingInterpolation();

        // Intake 2 chain
        toIntake2Zone = follower.pathBuilder()
                .addPath(new BezierLine(shoot1Pose, intake2StartPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(intake2StartPose, intake2EndPose))
                .setTangentHeadingInterpolation()
                .build();

        // Shoot 2 path
        toShoot2 = new Path(new BezierLine(intake2EndPose, shoot2Pose));
        toShoot2.setTangentHeadingInterpolation();

        // Intake 3 chain
        toIntake3Zone = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, intake3StartPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(intake3StartPose, intake3EndPose))
                .setTangentHeadingInterpolation()
                .build();

        // Shoot 3 path
        toShoot3 = new Path(new BezierLine(intake3EndPose, shoot3Pose));
        toShoot3.setTangentHeadingInterpolation();

        // End path
        toEnd = new Path(new BezierLine(shoot3Pose, endPose));
        toEnd.setTangentHeadingInterpolation();
    }

    // -------------------------------------------------------------------------
    //  Finite State Machine
    // -------------------------------------------------------------------------
    private void updateAutoState() {
        switch (autoState) {
            case WAIT_INITIAL_DELAY:
                intake.setPower(0.0);
                if (stateTimer.seconds() >= FIRST_SHOT_DELAY_SEC) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                if (feedState == FeedState.DONE) {
                    feeder.clutchOut();
                    feeder.armBlock();
                    intake.setPower(0.0);

                    follower.followPath(toIntake1Zone, false);
                    intakeRunning = false;
                    flywheelPrepped = false;
                    autoState = AutoState.DRIVE_TO_INTAKE1;
                }
                break;

            case DRIVE_TO_INTAKE1:
                if (!intakeRunning && distanceTo(intake1StartPose) < INTAKE_START_DIST) {
                    intakeTimer.reset();
                    intake.setPower(1.0);
                    intakeRunning = true;
                }
                if (!follower.isBusy()) {
                    // Safe Fallback: if distance check missed, reset timer right now upon arrival
                    if (!intakeRunning) {
                        intakeTimer.reset();
                        intake.setPower(1.0);
                        intakeRunning = true;
                    }
                    autoState = AutoState.INTAKE1;
                }
                break;

            case INTAKE1:
                if (intakeTimer.seconds() >= INTAKE_DURATION_SEC) {
                    intake.setPower(0.0);
                    intakeRunning = false;

                    follower.followPath(toShoot1, true);
                    flywheelPrepped = false;
                    autoState = AutoState.DRIVE_TO_SHOOT1;
                }
                break;

            case DRIVE_TO_SHOOT1:
                if (!flywheelPrepped && distanceTo(shoot1Pose) < FLYWHEEL_PREP_DIST) {
                    flywheelPrepTimer.reset();
                    flywheelPrepped = true;
                }
                if (!follower.isBusy()) {
                    // Safe Fallback: if arrival happens before the timer satisfies or triggers
                    if (!flywheelPrepped) {
                        flywheelPrepTimer.reset();
                        flywheelPrepped = true;
                    }
                    if (flywheelPrepTimer.seconds() >= FLYWHEEL_PREP_SEC) {
                        startFeedSequence();
                        autoState = AutoState.SHOOT1;
                    }
                }
                break;

            case SHOOT1:
                if (feedState == FeedState.DONE) {
                    feeder.clutchOut();
                    feeder.armBlock();

                    follower.followPath(toIntake2Zone, false);
                    intakeRunning = false;
                    flywheelPrepped = false;
                    autoState = AutoState.DRIVE_TO_INTAKE2;
                }
                break;

            case DRIVE_TO_INTAKE2:
                if (!intakeRunning && distanceTo(intake2StartPose) < INTAKE_START_DIST) {
                    intakeTimer.reset();
                    intake.setPower(1.0);
                    intakeRunning = true;
                }
                if (!follower.isBusy()) {
                    if (!intakeRunning) {
                        intakeTimer.reset();
                        intake.setPower(1.0);
                        intakeRunning = true;
                    }
                    autoState = AutoState.INTAKE2;
                }
                break;

            case INTAKE2:
                if (intakeTimer.seconds() >= INTAKE_DURATION_SEC) {
                    intake.setPower(0.0);
                    intakeRunning = false;

                    follower.followPath(toShoot2, true);
                    flywheelPrepped = false;
                    autoState = AutoState.DRIVE_TO_SHOOT2;
                }
                break;

            case DRIVE_TO_SHOOT2:
                if (!flywheelPrepped && distanceTo(shoot2Pose) < FLYWHEEL_PREP_DIST) {
                    flywheelPrepTimer.reset();
                    flywheelPrepped = true;
                }
                if (!follower.isBusy()) {
                    if (!flywheelPrepped) {
                        flywheelPrepTimer.reset();
                        flywheelPrepped = true;
                    }
                    if (flywheelPrepTimer.seconds() >= FLYWHEEL_PREP_SEC) {
                        startFeedSequence();
                        autoState = AutoState.SHOOT2;
                    }
                }
                break;

            case SHOOT2:
                if (feedState == FeedState.DONE) {
                    feeder.clutchOut();
                    feeder.armBlock();

                    follower.followPath(toIntake3Zone, false);
                    intakeRunning = false;
                    flywheelPrepped = false;
                    autoState = AutoState.DRIVE_TO_INTAKE3;
                }
                break;

            case DRIVE_TO_INTAKE3:
                if (!intakeRunning && distanceTo(intake3StartPose) < INTAKE_START_DIST) {
                    intakeTimer.reset();
                    intake.setPower(1.0);
                    intakeRunning = true;
                }
                if (!follower.isBusy()) {
                    if (!intakeRunning) {
                        intakeTimer.reset();
                        intake.setPower(1.0);
                        intakeRunning = true;
                    }
                    autoState = AutoState.INTAKE3;
                }
                break;

            case INTAKE3:
                if (intakeTimer.seconds() >= INTAKE_DURATION_SEC) {
                    intake.setPower(0.0);
                    intakeRunning = false;

                    follower.followPath(toShoot3, true);
                    flywheelPrepped = false;
                    autoState = AutoState.DRIVE_TO_SHOOT3;
                }
                break;

            case DRIVE_TO_SHOOT3:
                if (!flywheelPrepped && distanceTo(shoot3Pose) < FLYWHEEL_PREP_DIST) {
                    flywheelPrepTimer.reset();
                    flywheelPrepped = true;
                }
                if (!follower.isBusy()) {
                    if (!flywheelPrepped) {
                        flywheelPrepTimer.reset();
                        flywheelPrepped = true;
                    }
                    if (flywheelPrepTimer.seconds() >= FLYWHEEL_PREP_SEC) {
                        startFeedSequence();
                        autoState = AutoState.SHOOT3;
                    }
                }
                break;

            case SHOOT3:
                if (feedState == FeedState.DONE) {
                    feeder.clutchOut();
                    feeder.armBlock();
                    intake.setPower(0.0);

                    follower.followPath(toEnd, true);
                    autoState = AutoState.DRIVE_TO_END;
                }
                break;

            case DRIVE_TO_END:
                if (!follower.isBusy()) {
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                intake.setPower(0.0);
                turret.setAngle(0.0);
                flywheel.stop();
                break;
        }
    }

    // -------------------------------------------------------------------------
    //  Distance helper
    // -------------------------------------------------------------------------
    private double distanceTo(Pose target) {
        Pose current = follower.getPose();
        double dx = target.getX() - current.getX();
        double dy = target.getY() - current.getY();
        return Math.hypot(dx, dy);
    }

    // -------------------------------------------------------------------------
    //  Feed Sequence (Sub-State Machine)
    // -------------------------------------------------------------------------
    private void startFeedSequence() {
        feeder.armShoot();
        feeder.clutchIn();
        feedTimer.reset();
        feedState = FeedState.WAIT_BEFORE_INTAKE;
    }

    private void updateFeedSequence() {
        if (reversingIntake) {
            if (reverseTimer.seconds() < REVERSE_TIME_SEC) {
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
                intake.setPower(0.0);
                if (feedTimer.seconds() >= FEED_START_DELAY_SEC) {
                    intake.setPower(1.0);
                    feedState = FeedState.RUN_INTAKE;
                }
                break;

            case RUN_INTAKE:
                if (feedTimer.seconds() >= FEED_TOTAL_TIME_SEC) {
                    reversingIntake = true;
                    reverseTimer.reset();

                    intake.setPower(0.0);
                    feeder.clutchOut();
                    feedState = FeedState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    // -------------------------------------------------------------------------
    //  Alliance Helpers
    // -------------------------------------------------------------------------
    private double getTargetX() {
        return AllianceMirror.mirrorX(BLUE_TARGET_X, isRedAlliance);
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

    private double mirrorTurretCommand(double angleDeg) {
        return isRedAlliance ? -angleDeg : angleDeg;
    }

    // -------------------------------------------------------------------------
    //  Dynamic Turret Tracking — always looks at the goal
    // -------------------------------------------------------------------------
    private void trackGoalFromOdometry(Pose pose) {
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double turretX = robotX - TURRET_CENTER_OFFSET_IN * Math.cos(robotHeadingRad);
        double turretY = robotY - TURRET_CENTER_OFFSET_IN * Math.sin(robotHeadingRad);

        double dx = getTargetX() - turretX;
        double dy = TARGET_Y - turretY;

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
}



//helloooo