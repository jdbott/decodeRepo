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
import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "Far Auto Simple")
public class V3FarAuto extends LinearOpMode {

    private Servo hoodServo;
    private Servo armServo;
    private Servo clutchServo;

    private Follower follower;
    private Turret turret;
    private DcMotorEx intakeMotor;
    private FlywheelASG flywheel;

    private boolean isRedAlliance = false;

    // ===== Fixed shot settings =====
    private static final double FIXED_HOOD_DEG = 55;
    private static final double FIXED_FLYWHEEL_RAD = 455;

    // ===== Field positions (BLUE-NATIVE) =====
    private static final double START_X = 64.1;
    private static final double START_Y = 6.75;
    private static final double START_HEADING_DEG = 180.0;

    private static final double BLUE_TARGET_X = 5.0;
    private static final double TARGET_Y = 139.0;

    private static final double SHOOT2_Y_OFFSET = 5.5;

    private static final double LAST_LINE_X = 12.5;
    private static final double LAST_LINE_Y = 38.0;

    private static final double INTAKE_1_X = 15.0;
    private static final double INTAKE_1_Y = 9.5;

    private static final double BACKUP_DISTANCE = 10.0;
    private static final double ALT_CYCLE_Y_OFFSET = 19.0;

    // ===== Turret config =====
    private static final double TURRET_CENTER_OFFSET_IN = 1.5;
    private static final double TURRET_MIN_DEG = -180.0;
    private static final double TURRET_MAX_DEG = 180.0;
    private static final double TURRET_OFFSET_DEG = 180;

    // ===== Timing =====
    private static final double FIRST_SHOT_DELAY_SEC = 2.8;
    private static final double FEED_START_DELAY_SEC = 0.10;
    private static final double FEED_TOTAL_TIME_SEC = 1.00;
    private static final double REVERSE_TIME_SEC = 0.25;

    // ===== Fixed turret aim commands (BLUE-NATIVE) =====
    private static final double INIT_TURRET_ANGLE_DEG = 112.0;
    private static final double RUN_TURRET_ANGLE_DEG = 112.0;

    // ===== Poses =====
    private Pose startPose;
    private Pose shootPose1;
    private Pose shootPose2;
    private Pose lastLinePose;

    // ===== Paths =====
    private Path toLastLine;
    private Path fromLastLineToShoot2;

    private Path toIntake1;
    private Path backupFromIntake1;
    private Path backToShoot2;

    // ===== Timers =====
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime reverseTimer = new ElapsedTime();

    private boolean reversingIntake = false;

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
        SHOOT_FIRST,

        DRIVE_TO_LAST_LINE,
        DRIVE_BACK_TO_SHOOT2,
        SHOOT_SECOND,

        DRIVE_TO_INTAKE1,
        BACK_UP_FROM_INTAKE1,
        DRIVE_BACK_TO_SHOOT_AGAIN,
        SHOOT_REPEAT,

        DONE
    }

    private AutoState autoState = AutoState.WAIT_INITIAL_DELAY;

    @Override
    public void runOpMode() {
        isRedAlliance = AllianceStore.isRed(hardwareMap.appContext);

        // ----- Poses -----
        startPose = p(START_X, START_Y, START_HEADING_DEG);
        shootPose1 = p(START_X, START_Y, 180.0);
        shootPose2 = p(START_X, START_Y + SHOOT2_Y_OFFSET, 180.0);
        lastLinePose = p(LAST_LINE_X, LAST_LINE_Y, 180.0);

        // ----- Hardware -----
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

        buildBasePaths();

        // ----- Initial mechanism state -----
        intakeMotor.setPower(0.0);
        flywheel.stop();
        clutchIn();
        armBlock();
        setHoodAngle(FIXED_HOOD_DEG);

        telemetry.addLine("Far Auto Simple Initialized");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Start Pose", startPose);
        telemetry.addData("Shoot Pose 2", shootPose2);
        telemetry.update();

        // ----- Init loop -----
        while (opModeInInit()) {
            follower.updatePose();
            turret.setAngle(mirrorTurretCommand(INIT_TURRET_ANGLE_DEG));
            turret.update();

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Turret Angle", turret.getCurrentAngle());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // ----- On start -----
        setHoodAngle(FIXED_HOOD_DEG);
        flywheel.setTargetVelocity(FIXED_FLYWHEEL_RAD);

        stateTimer.reset();
        autoState = AutoState.WAIT_INITIAL_DELAY;

        while (opModeIsActive()) {
            follower.update();

            if (autoState != AutoState.DONE) {
                // trackGoalFromOdometry(follower.getPose());
                turret.setAngle(mirrorTurretCommand(RUN_TURRET_ANGLE_DEG));
            } else {
                turret.setAngle(0.0);
            }

            turret.update();
            flywheel.setTargetVelocity(FIXED_FLYWHEEL_RAD);
            flywheel.update();

            updateFeedSequence();
            updateAutoState();

            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Auto State", autoState);
            telemetry.addData("Feed State", feedState);
            telemetry.addData("Cycle Count", extraCycleCount);
            telemetry.addData("Using High Intake", useHighIntakeCycle());
            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Turret Angle", turret.getCurrentAngle());
            telemetry.addData("Flywheel Target", FIXED_FLYWHEEL_RAD);
            telemetry.addData("Flywheel Actual", flywheel.getVelocityRadPerSec());
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

    private double mirrorTurretCommand(double angleDeg) {
        return isRedAlliance ? -angleDeg : angleDeg;
    }

    private void buildBasePaths() {
        toLastLine = new Path(
                new BezierCurve(
                        shootPose1,
                        p(45.0, 38.0),
                        lastLinePose
                )
        );
        toLastLine.setConstantHeadingInterpolation(h(180.0));

        fromLastLineToShoot2 = new Path(
                new BezierLine(
                        lastLinePose,
                        shootPose2
                )
        );
        fromLastLineToShoot2.setConstantHeadingInterpolation(h(180.0));
        fromLastLineToShoot2.setBrakingStrength(0.85);
    }

    private boolean useHighIntakeCycle() {
        // cycle 0 = normal, 1 = high, 2 = normal, 3 = high
        return (extraCycleCount % 2) == 1;
    }

    private Pose getCurrentIntakePose() {
        double yOffset = useHighIntakeCycle() ? ALT_CYCLE_Y_OFFSET : 0.0;
        return p(
                INTAKE_1_X,
                INTAKE_1_Y + yOffset,
                180.0
        );
    }

    private Pose getCurrentBackedPose() {
        double yOffset = useHighIntakeCycle() ? ALT_CYCLE_Y_OFFSET : 0.0;
        return p(
                INTAKE_1_X + BACKUP_DISTANCE,
                INTAKE_1_Y + yOffset,
                180.0
        );
    }

    private void buildCyclePathsForCurrentCycle() {
        Pose currentIntakePose = getCurrentIntakePose();
        Pose currentBackedPose = getCurrentBackedPose();

        toIntake1 = new Path(
                new BezierLine(
                        shootPose2,
                        currentIntakePose
                )
        );
        toIntake1.setConstantHeadingInterpolation(h(200.0));

        backupFromIntake1 = new Path(
                new BezierLine(
                        currentIntakePose,
                        currentBackedPose
                )
        );
        backupFromIntake1.setConstantHeadingInterpolation(h(180.0));

        backToShoot2 = new Path(
                new BezierLine(
                        currentBackedPose,
                        shootPose2
                )
        );
        backToShoot2.setConstantHeadingInterpolation(h(180.0));
        backToShoot2.setBrakingStrength(0.85);
    }

    private void updateAutoState() {
        switch (autoState) {
            case WAIT_INITIAL_DELAY:
                intakeMotor.setPower(0.0);
                if (stateTimer.seconds() >= FIRST_SHOT_DELAY_SEC) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_FIRST;
                }
                break;

            case SHOOT_FIRST:
                if (feedState == FeedState.DONE) {
                    clutchOut();
                    armBlock();
                    intakeMotor.setPower(0.0);

                    follower.followPath(toLastLine, false);
                    autoState = AutoState.DRIVE_TO_LAST_LINE;
                }
                break;

            case DRIVE_TO_LAST_LINE:
                if (!follower.isBusy()) {
                    follower.followPath(fromLastLineToShoot2, true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT2;
                }
                break;

            case DRIVE_BACK_TO_SHOOT2:
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_SECOND;
                }
                break;

            case SHOOT_SECOND:
                if (feedState == FeedState.DONE) {
                    clutchOut();
                    armBlock();

                    buildCyclePathsForCurrentCycle();
                    follower.followPath(toIntake1, false);
                    intakeMotor.setPower(1.0);
                    autoState = AutoState.DRIVE_TO_INTAKE1;
                }
                break;

            case DRIVE_TO_INTAKE1:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    follower.followPath(backupFromIntake1, true);
                    autoState = AutoState.BACK_UP_FROM_INTAKE1;
                }
                break;

            case BACK_UP_FROM_INTAKE1:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    follower.followPath(backToShoot2, true);
                    autoState = AutoState.DRIVE_BACK_TO_SHOOT_AGAIN;
                }
                break;

            case DRIVE_BACK_TO_SHOOT_AGAIN:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_REPEAT;
                }
                break;

            case SHOOT_REPEAT:
                if (feedState == FeedState.DONE) {
                    clutchOut();
                    armBlock();

                    extraCycleCount++;

                    if (extraCycleCount < 4) {
                        buildCyclePathsForCurrentCycle();
                        follower.followPath(toIntake1, false);
                        intakeMotor.setPower(1.0);
                        autoState = AutoState.DRIVE_TO_INTAKE1;
                    } else {
                        intakeMotor.setPower(0.0);
                        autoState = AutoState.DONE;

                        Pose currentPose = follower.getPose();
                        double retreatDx = isRedAlliance ? 23 : -23.0;

                        follower.followPath(
                                new Path(
                                        new BezierLine(
                                                currentPose,
                                                new Pose(currentPose.getX() + retreatDx, currentPose.getY())
                                        )
                                ),
                                true
                        );
                    }
                }
                break;

            case DONE:
                intakeMotor.setPower(0.0);
                turret.setAngle(0.0);
                break;
        }
    }

    private void startFeedSequence() {
        armShoot();
        clutchIn();
        feedTimer.reset();
        feedState = FeedState.WAIT_BEFORE_INTAKE;
    }

    private void updateFeedSequence() {
        if (reversingIntake) {
            if (reverseTimer.seconds() < REVERSE_TIME_SEC) {
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
                intakeMotor.setPower(0.0);
                if (feedTimer.seconds() >= FEED_START_DELAY_SEC) {
                    intakeMotor.setPower(1.0);
                    feedState = FeedState.RUN_INTAKE;
                }
                break;

            case RUN_INTAKE:
                if (feedTimer.seconds() >= FEED_TOTAL_TIME_SEC) {
                    reversingIntake = true;
                    reverseTimer.reset();

                    intakeMotor.setPower(0.0);
                    clutchOut();
                    feedState = FeedState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

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

    public void clutchIn() {
        clutchServo.setPosition(0.48);
    }

    public void clutchOut() {
        clutchServo.setPosition(0.52);
    }

    public void armBlock() {
        armServo.setPosition(0.26);
    }

    public void armShoot() {
        armServo.setPosition(0.395);
    }

    public void setHoodAngle(double angleDeg) {
        final double MIN_ANGLE = 30.0;
        final double MAX_ANGLE = 60.0;

        final double MIN_POS = 0.395;
        final double MAX_POS = 0.9;

        double clippedAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDeg));
        double t = (clippedAngle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        double pos = MIN_POS + t * (MAX_POS - MIN_POS);
        pos = Math.max(MIN_POS, Math.min(MAX_POS, pos));

        hoodServo.setPosition(pos);
    }
}