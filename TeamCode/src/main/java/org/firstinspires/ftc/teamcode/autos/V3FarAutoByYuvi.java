package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.AllianceMirror;
import org.firstinspires.ftc.teamcode.AllianceStore;
import org.firstinspires.ftc.teamcode.AutoStartStore;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.hardwareClasses.Feeder;
import org.firstinspires.ftc.teamcode.hardwareClasses.Flywheel;
import org.firstinspires.ftc.teamcode.hardwareClasses.Hood;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "V3 Far Auto : Yuvi Edition", group = "Autonomous")
public class V3FarAutoByYuvi extends OpMode {

    /* ===== Hardware Subsystems ===== */
    private Follower follower;
    private Turret turret;
    private Flywheel flywheel;
    private Intake intake;
    private Hood hood;
    private Feeder feeder;

    /* ===== Fixed Shot Settings ===== */
    private static final double FIXED_HOOD_DEG = 53.5;
    private static final double FIXED_FLYWHEEL_RAD = 445.0;
    private static final double INIT_TURRET_ANGLE_DEG = 113.0;
    private static final double RUN_TURRET_ANGLE_DEG = 113.0;

    /* ===== Gate Collection Point ===== */
    private static final double GATE_X = 136.5559649236386;
    private static final double GATE_Y = 69.90375693752594;
    private static final double GATE_INTAKE_THRESHOLD = 20.0; // inches before arrival

    /* ===== Timing ===== */
    private static final double FIRST_SHOT_DELAY_SEC = 2.5;
    private static final double SHOOT_PAUSE_MS = 1600.0;   // Standard shooting stops
    private static final double GATE_WAIT_MS = 2400.0;     // Gate collection (no shot)
    private static final double FLYWHEEL_TOLERANCE = 10.0;

    private static final double FEED_START_DELAY_SEC = 0.10;
    private static final double FEED_TOTAL_TIME_SEC = 1.00;
    private static final double REVERSE_TIME_SEC = 0.25;

    /* ===== Timers ===== */
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime reverseTimer = new ElapsedTime();

    /* ===== State ===== */
    private boolean isRedAlliance = false;
    private boolean reversingIntake = false;

    private enum AutoState {
        WAIT_INITIAL_DELAY,
        SHOOT_FIRST,
        DRIVE_SEG1, PAUSE_SEG1,
        DRIVE_SEG2, PAUSE_SEG2,
        DRIVE_SEG3, PAUSE_SEG3,
        DRIVE_SEG4, PAUSE_SEG4,
        DRIVE_SEG5, PAUSE_SEG5,   // Gate approach & collection
        DRIVE_SEG6, PAUSE_SEG6,
        DRIVE_SEG7, PAUSE_SEG7,
        DRIVE_SEG8, PAUSE_SEG8,
        DONE
    }

    private enum FeedState {
        IDLE,
        WAIT_BEFORE_INTAKE,
        RUN_INTAKE,
        DONE
    }

    private AutoState autoState = AutoState.WAIT_INITIAL_DELAY;
    private FeedState feedState = FeedState.IDLE;

    /* ===== Segmented Path Chains ===== */
    private PathChain seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8;

    /* ===================================================================== */
    /*  LIFECYCLE                                                            */
    /* ===================================================================== */

    @Override
    public void init() {
        isRedAlliance = AllianceStore.isRed(hardwareMap.appContext);
        AutoStartStore.setFar(hardwareMap.appContext);

        /* -- Pedro Pathing -- */
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = p(72.0, 8.0, 90.0);
        follower.setStartingPose(startPose);
        follower.updatePose();
        follower.setMaxPower(1.0);

        /* -- Hardware -- */
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        feeder = new Feeder(hardwareMap);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new Flywheel(hardwareMap, battery);

        turret = new Turret(hardwareMap, RobotConfig.TURRET_MOTOR, DcMotorSimple.Direction.REVERSE);

        buildPaths();

        /* -- Safe defaults (exact from V3FarAuto) -- */
        intake.setPower(0.0);
        flywheel.stop();
        feeder.clutchIn();
        feeder.armBlock();
        hood.setAngle(FIXED_HOOD_DEG);

        telemetry.addLine("Far Auto Perfect Initialized");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.updatePose();
        turret.setAngle(mirrorTurretCommand(INIT_TURRET_ANGLE_DEG));
        turret.update();

        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Turret Angle", turret.getCurrentAngle());
        telemetry.update();
    }

    @Override
    public void start() {
        stateTimer.reset();
        autoState = AutoState.WAIT_INITIAL_DELAY;

        hood.setAngle(FIXED_HOOD_DEG);
        flywheel.setTargetVelocity(FIXED_FLYWHEEL_RAD);
    }

    @Override
    public void loop() {
        /* -- Frame-rate decoupled updates -- */
        follower.update();
        flywheel.setTargetVelocity(FIXED_FLYWHEEL_RAD);
        flywheel.update();
        updateFeedSequence();

        /* -- Fixed turret aim (exact from V3FarAuto) -- */
        if (autoState != AutoState.DONE) {
            turret.setAngle(mirrorTurretCommand(RUN_TURRET_ANGLE_DEG));
        } else {
            turret.setAngle(0.0);
        }
        turret.update();

        /* -- Main state machine -- */
        updateAutoState();

        /* -- Telemetry -- */
        telemetry.addData("State", autoState);
        telemetry.addData("Feed", feedState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Flywheel", flywheel.getVelocityRadPerSec());
        telemetry.addData("Turret", turret.getCurrentAngle());
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.update();
    }

    /* ===================================================================== */
    /*  PATH CONSTRUCTION  (Exact uploaded Pedro Pathing)                    */
    /* ===================================================================== */

    private void buildPaths() {
        // Seg 1 – initialization push curve
        seg1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(83.458, 8.000), p(141.143, 39.956), p(140.594, 6.691),
                        p(138.580, 16.068), p(138.580, 7.013), p(138.081, 3.240)))
                .setTangentHeadingInterpolation()
                .build();

        // Seg 2 – base return line
        seg2 = follower.pathBuilder()
                .addPath(new BezierLine(p(138.081, 3.240), p(84.930, 5.641)))
                .setTangentHeadingInterpolation()
                .build();

        // Seg 3 – deep field push curve (BALL RICH ZONE)
        seg3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(84.930, 5.641), p(97.822, 64.859), p(123.669, 58.869)))
                .setTangentHeadingInterpolation()
                .build();

        // Seg 4 – primary reverse sweep
        seg4 = follower.pathBuilder()
                .addPath(new BezierLine(p(123.669, 58.869), p(87.495, 91.064)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Seg 5 – outbound sweep to gate
        seg5 = follower.pathBuilder()
                .addPath(new BezierLine(p(87.495, 91.064), p(136.556, 69.904)))
                .setLinearHeadingInterpolation(h(0.0), h(25.0))
                .build();

        // Seg 6 – return from gate
        seg6 = follower.pathBuilder()
                .addPath(new BezierLine(p(136.556, 69.904), p(91.113, 93.969)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Seg 7 – complex clearance curve (BALL RICH ZONE)
        seg7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        p(91.113, 93.969), p(99.017, 27.747), p(112.224, 34.252),
                        p(118.011, 35.520), p(123.634, 34.352)))
                .setLinearHeadingInterpolation(h(0.0), h(355.0))
                .build();

        // Seg 8 – final park line
        seg8 = follower.pathBuilder()
                .addPath(new BezierLine(p(123.634, 34.352), p(70.553, 21.301)))
                .setLinearHeadingInterpolation(h(0.0), h(0.0))
                .build();
    }

    /* ===================================================================== */
    /*  STATE MACHINE                                                        */
    /* ===================================================================== */

    private void updateAutoState() {
        switch (autoState) {

            case WAIT_INITIAL_DELAY:
                intake.setPower(0.0);
                if (stateTimer.seconds() >= FIRST_SHOT_DELAY_SEC
                        && Math.abs(flywheel.getVelocityRadPerSec() - FIXED_FLYWHEEL_RAD) < FLYWHEEL_TOLERANCE) {
                    startFeedSequence();
                    autoState = AutoState.SHOOT_FIRST;
                }
                break;

            case SHOOT_FIRST:
                if (feedState == FeedState.DONE) {
                    resetMechanisms();
                    follower.followPath(seg1, true);
                    autoState = AutoState.DRIVE_SEG1;
                }
                break;

            case DRIVE_SEG1:
                if (!follower.isBusy()) { stateTimer.reset(); autoState = AutoState.PAUSE_SEG1; }
                break;
            case PAUSE_SEG1:
                handleShootingPause(seg2, AutoState.DRIVE_SEG2);
                break;

            case DRIVE_SEG2:
                if (!follower.isBusy()) { stateTimer.reset(); autoState = AutoState.PAUSE_SEG2; }
                break;
            case PAUSE_SEG2:
                handleShootingPause(seg3, AutoState.DRIVE_SEG3);
                break;

            /* ===== DRIVE_SEG3: Deep field push — INTAKE ON to collect balls ===== */
            case DRIVE_SEG3:
                intake.setPower(1.0); // collect balls while pushing through field
                if (!follower.isBusy()) {
                    intake.setPower(0.0);
                    stateTimer.reset();
                    autoState = AutoState.PAUSE_SEG3;
                }
                break;
            case PAUSE_SEG3:
                handleShootingPause(seg4, AutoState.DRIVE_SEG4);
                break;

            case DRIVE_SEG4:
                if (!follower.isBusy()) { stateTimer.reset(); autoState = AutoState.PAUSE_SEG4; }
                break;
            case PAUSE_SEG4:
                handleShootingPause(seg5, AutoState.DRIVE_SEG5);
                break;

            /* ===== DRIVE_SEG5: To gate — INTAKE ON near gate ===== */
            case DRIVE_SEG5:
                double dxGate = GATE_X - follower.getPose().getX();
                double dyGate = GATE_Y - follower.getPose().getY();
                if (Math.hypot(dxGate, dyGate) < GATE_INTAKE_THRESHOLD) {
                    intake.setPower(1.0);
                }
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    autoState = AutoState.PAUSE_SEG5;
                }
                break;

            /* ===== PAUSE_SEG5: Gate wait — 2.5s collection, NO shooting ===== */
            case PAUSE_SEG5:
                intake.setPower(1.0); // stay on to collect from gate
                if (stateTimer.milliseconds() >= GATE_WAIT_MS) {
                    intake.setPower(0.0);
                    resetMechanisms();
                    follower.followPath(seg6, true);
                    autoState = AutoState.DRIVE_SEG6;
                }
                break;

            case DRIVE_SEG6:
                if (!follower.isBusy()) { stateTimer.reset(); autoState = AutoState.PAUSE_SEG6; }
                break;
            case PAUSE_SEG6:
                handleShootingPause(seg7, AutoState.DRIVE_SEG7);
                break;

            /* ===== DRIVE_SEG7: Clearance curve — INTAKE ON to collect balls ===== */
            case DRIVE_SEG7:
                intake.setPower(1.0); // collect balls on clearance sweep
                if (!follower.isBusy()) {
                    intake.setPower(0.0);
                    stateTimer.reset();
                    autoState = AutoState.PAUSE_SEG7;
                }
                break;
            case PAUSE_SEG7:
                handleShootingPause(seg8, AutoState.DRIVE_SEG8);
                break;

            case DRIVE_SEG8:
                if (!follower.isBusy()) { stateTimer.reset(); autoState = AutoState.PAUSE_SEG8; }
                break;
            case PAUSE_SEG8:
                verifyFlywheelAndShoot();
                if (stateTimer.milliseconds() >= SHOOT_PAUSE_MS && feedState == FeedState.DONE) {
                    resetMechanisms();
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                intake.setPower(0.0);
                flywheel.stop();
                turret.setAngle(0.0);
                break;
        }
    }

    /** 1.5 s verified-velocity shooting pause. */
    private void handleShootingPause(PathChain nextSeg, AutoState nextState) {
        boolean shotComplete = verifyFlywheelAndShoot();
        if (stateTimer.milliseconds() >= SHOOT_PAUSE_MS && shotComplete) {
            resetMechanisms();
            if (nextSeg != null) follower.followPath(nextSeg, true);
            autoState = nextState;
        }
    }

    /** Only starts feeding once flywheel is within tolerance. Returns true when feed is done. */
    private boolean verifyFlywheelAndShoot() {
        double err = Math.abs(flywheel.getVelocityRadPerSec() - FIXED_FLYWHEEL_RAD);
        if (err < FLYWHEEL_TOLERANCE && feedState == FeedState.IDLE) {
            startFeedSequence();
            return false;
        }
        return feedState == FeedState.DONE;
    }

    /* ===================================================================== */
    /*  FEED STATE MACHINE  (exact from V3FarAuto)                            */
    /* ===================================================================== */

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
            case DONE:
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
        }
    }

    private void resetMechanisms() {
        feeder.clutchOut();
        feeder.armBlock();
        intake.setPower(0.0);
        feedState = FeedState.IDLE;
    }

    /* ===================================================================== */
    /*  ALLIANCE MIRRORING  (exact from V3FarAuto)                            */
    /* ===================================================================== */

    private double mirrorTurretCommand(double angleDeg) {
        return isRedAlliance ? -angleDeg : angleDeg;
    }

    private Pose p(double x, double y) {
        return AllianceMirror.mirrorPose(new Pose(x, y, 0), isRedAlliance);
    }

    private Pose p(double x, double y, double headingDeg) {
        return AllianceMirror.mirrorPose(new Pose(x, y, Math.toRadians(headingDeg)), isRedAlliance);
    }

    private double h(double headingDeg) {
        return Math.toRadians(AllianceMirror.mirrorHeadingDeg(headingDeg, isRedAlliance));
    }
}