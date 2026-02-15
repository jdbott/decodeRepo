package org.firstinspires.ftc.teamcode.teles;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwareClasses.BasePlateFast;
import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Gantry;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.Locale;

@TeleOp(name = "V2 TELEOP BLUE")
public class V2TeleBlue extends LinearOpMode {

    // =========================================================
    // Subsystems / hardware
    // =========================================================
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlateFast basePlate;

    private FlywheelASG flywheelASG;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private final Turret turret = new Turret();

    // Edge memory for basePlate LB trigger
    private boolean lbPrev = false;

    // =========================================================
    // Limelight
    // =========================================================
    private Limelight3A limelight3A;
    private int llLastSeenId = -1;
    private double llLastTxDeg = Double.NaN;
    private boolean llUsingTxAim = false;

    // TX hold (anti-NaN jitter)
    private double llLastGoodTxDeg = Double.NaN;
    private long llLastGoodTxTimeMs = 0;
    private static final long LL_TX_HOLD_MS = 250;

    // =========================================================
    // Intake FSM
    // =========================================================
    private enum IntakeState {
        OFF,
        ON,
        SHUTDOWN_WAIT, // gate down now; wait 0.4s then stop intake motor
        REVERSE
    }

    private IntakeState intakeState = IntakeState.OFF;
    private boolean intakeToggleActive = false;
    private boolean lastSquareButton = false;
    private long intakeShutdownStartMs = 0;

    private static final long INTAKE_SHUTDOWN_DELAY_MS = 400;

    private long intakeReverseStartMs = 0;
    private static final long INTAKE_REVERSE_MS = 500; // example

    // =========================================================
    // Gate toggle (RB)
    // =========================================================
    private boolean rbPrev = false;
    private boolean gateToggleAllTheWayUp = true; // true = all-the-way-up, false = "one position"

    // =========================================================
    // Turret state fields
    // =========================================================
    private boolean turretShootingActive = false;
    private boolean turretLastShootToggleBtn = false;

    private boolean turretManualOverride = false;
    private long turretLastManualInputMs = 0;

    private boolean sharePrev = false;

    // =========================================================
    // Target (for odom aim + odom distance)
    // =========================================================
    private static double TARGET_X = 7;
    private static double TARGET_Y = 138;

    // =========================================================
    // Flywheel tuning (RPM on gamepad1)
    // =========================================================
    private double flywheelTuneRPM = 3000;

    private boolean gp1DpadUpPrev = false;
    private boolean gp1DpadDownPrev = false;

    private static final double FLYWHEEL_RPM_STEP = 50.0;
    private static final double FLYWHEEL_RPM_MIN = 0.0;
    private static final double FLYWHEEL_RPM_MAX = 6000.0;

    // For telemetry: what we commanded this frame
    private double lastCmdRPM = 0.0;
    private double lastCmdRadPerSec = 0.0;

    // =========================================================
    // Turret idle hold (NO SNAP ON START)
    // =========================================================
    private boolean turretIdleHoldEnabled = true; // start in "don't move / hold current" mode
    private double turretIdleHoldAngleDeg = Double.NaN;

    private double turretOffset = 3;

    // =========================================================
    // Distance + Flywheel Auto Config (TUNE ALL HERE)
    // =========================================================

    // --- Units ---
    private static final double METERS_TO_INCHES = 39.3701;

    // --- Angle/trig distance constants (MEASURE THESE) ---
    private static final double CAM_HEIGHT_M = 0.32;   // camera lens height from floor (meters)
    private static final double CAM_PITCH_DEG = 25.0;  // camera mounting pitch angle up from horizontal (deg)
    private static final double TARGET_HEIGHT_M = 1.04; // height of the feature you're aiming at (meters)

    // --- Flywheel fixed speeds ---
    private static final double FAR_FIXED_RPM = 3700.0;

    // --- CLOSE zone (ODOM DISTANCE ONLY) ---
    private static final double CLOSE_MIN_DIST_IN = 49.0;     // lower bound of your dataset
    private static final double CLOSE_FLAT_END_IN = 72.4;     // <= this: 2900 RPM
    private static final double CLOSE_CUBIC_START_IN = 72.5;  // >= this: cubic regression
    private static final double CLOSE_MAX_DIST_IN = 95.0;     // upper bound of your dataset

    private static final double CLOSE_FLAT_RPM = 2900.0;      // flat region RPM
    private static final double CLOSE_MAX_RPM = 3120.0;       // max region clamp RPM

    // Flywheel mode state
    private enum FlywheelMode { CLOSE, FAR }
    private FlywheelMode flywheelMode = FlywheelMode.CLOSE; // default start: FAR
    private boolean flywheelAutoEnabled = true;            // default ON

    private boolean gp2DpadUpPrev = false;
    private boolean gp2DpadDownPrev = false;
    private boolean gp2OptionsPrev = false;

    // Distance telemetry values (all inches)
    private double distCamZIn = Double.NaN;
    private double distAngleIn = Double.NaN;
    private double distOdomIn = Double.NaN;
    private double distAutoUsedIn = Double.NaN;

    // Legacy telemetry field you were already showing
    private double lastDistanceToTargetIn = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // -----------------------------
        // Follower init
        // -----------------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25, 85, Math.toRadians(180)));
        follower.updatePose();
        follower.setMaxPower(1);
        follower.startTeleOpDrive();

        // -----------------------------
        // Hardware map (drivetrain)
        // -----------------------------
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

        // -----------------------------
        // Subsystems
        // -----------------------------
        intake = new Intake(hardwareMap);
        gantry = new Gantry(hardwareMap);
        basePlate = new BasePlateFast(hardwareMap);

        // -----------------------------
        // FlywheelASG init (LIKE AUTO)
        // -----------------------------
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheelASG = new FlywheelASG(hardwareMap, battery);
        flywheelASG.setTargetVelocity(0.0);

        // Safe defaults
        intake.intakeStop();
        gantry.moveGantryToPos("back");
        basePlate.rampBack();
        basePlate.frontPopperDown();
        basePlate.middlePopperDown();
        basePlate.cancelShootAndReset();

        // -----------------------------
        // Limelight init
        // -----------------------------
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(9);
        limelight3A.setPollRateHz(100);
        limelight3A.start();

        // -----------------------------
        // IMU init
        // -----------------------------
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
        imu.resetYaw();

        // -----------------------------
        // Turret init
        // -----------------------------
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.update();

        telemetry.addLine("Initialized. Press START to begin.");
        telemetry.update();

        while (opModeInInit()) {
            turret.update();
            basePlate.update();
            flywheelASG.update();
        }

        waitForStart();
        resetRuntime();

        // Latch turret's starting position so it DOES NOT snap to a park angle
        turretIdleHoldAngleDeg = turret.getCurrentAngle();
        turret.setAngle(turretIdleHoldAngleDeg);

        while (opModeIsActive()) {

            follower.update();
            Pose pose = follower.getPose();

            // -----------------------------
            // Drive
            // -----------------------------
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());
            drive(robotHeadingDeg);

            // -----------------------------
            // Flywheel mode select + manual tuning
            // -----------------------------
            handleFlywheelModeSelect();
            handleDriverFlywheelTuning();

            // -----------------------------
            // Distances (all in inches)
            // -----------------------------
            distOdomIn = computeDistanceToTargetInches(pose);

            // Preserve your old telemetry variable
            lastDistanceToTargetIn = distOdomIn;

            // -----------------------------
            // Intake toggle + FSM
            // -----------------------------
            handleIntakeToggleAndFSM();

            // -----------------------------
            // Gate toggle on RB
            // -----------------------------
            handleGateToggle();

            // -----------------------------
            // Turret logic + Limelight goal tracking
            // (also sets flywheel target)
            // -----------------------------
            handleTurretTrackingAndControl(pose);

            // -----------------------------
            // BasePlate controls
            // -----------------------------
            boolean lbNow = gamepad2.left_bumper;
            if (lbNow && !lbPrev) {
                basePlate.startShootFromPrep();
            }
            lbPrev = lbNow;

            // Update subsystems
            basePlate.update();
            turret.update();
            flywheelASG.update();

            // -----------------------------
            // Telemetry
            // -----------------------------
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.addData("IntakeState", intakeState);
            telemetry.addData("GateMode", gateToggleAllTheWayUp ? "ALL_UP" : "ONE_POS");
            telemetry.addData("ShootMode", turretShootingActive);

            telemetry.addData("TurretIdleHold", turretIdleHoldEnabled);
            telemetry.addData("TurretIdleHoldDeg", Double.isFinite(turretIdleHoldAngleDeg)
                    ? String.format(Locale.US, "%.2f", turretIdleHoldAngleDeg)
                    : "NaN");

            telemetry.addData("LL_UsingTxAim", llUsingTxAim);
            telemetry.addData("LL_LastId", llLastSeenId);
            telemetry.addData("LL_TxDeg", Double.isNaN(llLastTxDeg) ? "NaN" : String.format(Locale.US, "%.2f", llLastTxDeg));

            telemetry.addData("DistOdom(in)", Double.isFinite(distOdomIn) ? String.format(Locale.US, "%.2f", distOdomIn) : "NaN");
            telemetry.addData("DistAutoUsed(in)", Double.isFinite(distAutoUsedIn) ? String.format(Locale.US, "%.2f", distAutoUsedIn) : "NaN");

            telemetry.addData("FlywheelMode", flywheelMode);
            telemetry.addData("AutoEnabled", flywheelAutoEnabled);

            telemetry.addData("TuneRPM(gamepad1)", String.format(Locale.US, "%.0f", flywheelTuneRPM));
            telemetry.addData("FlywheelCmdRPM", String.format(Locale.US, "%.0f", lastCmdRPM));
            telemetry.addData("FlywheelCmdRadS", String.format(Locale.US, "%.2f", lastCmdRadPerSec));
            telemetry.addData("turret", turret.getCurrentAngle());

            // =====================
// HEADING DEBUG
// =====================
            double followerHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
            double imuYawDeg = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);

            telemetry.addLine("=== HEADING DEBUG ===");
            telemetry.addData("Follower Heading (deg)",
                    String.format(Locale.US, "%.2f", followerHeadingDeg));
            telemetry.addData("IMU Yaw (deg)",
                    String.format(Locale.US, "%.2f", imuYawDeg));
            telemetry.addData("Heading Delta (Follower - IMU)",
                    String.format(Locale.US, "%.2f", normalize180(followerHeadingDeg - imuYawDeg)));

            telemetry.update();
        }
    }

    // =========================================================
    // Intake toggle + FSM
    // =========================================================
    private void handleIntakeToggleAndFSM() {
        boolean squareButton = gamepad2.square;
        if (squareButton && !lastSquareButton) {
            intakeToggleActive = !intakeToggleActive;

            if (intakeToggleActive) {
                intakeState = IntakeState.ON;
                basePlate.cancelShootAndReset();
            } else {
                intakeState = IntakeState.SHUTDOWN_WAIT;
                intakeShutdownStartMs = System.currentTimeMillis();
            }
        }
        lastSquareButton = squareButton;

        switch (intakeState) {
            case OFF:
                intake.intakeStop();
                break;

            case ON:
                gantry.moveGantryToPos("middle");
                intake.intakeIn();
                gateAllTheWayUp();
                break;

            case SHUTDOWN_WAIT:
                intake.intakeStop();
                gateDown();
                gantry.moveGantryToPos("back");

                if (System.currentTimeMillis() - intakeShutdownStartMs >= INTAKE_SHUTDOWN_DELAY_MS) {
                    gateDown();
                    intakeReverseStartMs = System.currentTimeMillis();
                    intakeState = IntakeState.REVERSE;
                    intake.intakeOut();
                    basePlate.prepShootOnly();
                }
                break;

            case REVERSE:
                intake.intakeOut();
                if (System.currentTimeMillis() - intakeReverseStartMs >= INTAKE_REVERSE_MS) {
                    intakeState = IntakeState.OFF;
                    intake.intakeStop();
                }
                break;
        }
    }

    // =========================================================
    // Gate toggle (RB)
    // =========================================================
    private void handleGateToggle() {
        if (intakeState == IntakeState.ON || intakeState == IntakeState.SHUTDOWN_WAIT) return;

        boolean rbNow = gamepad2.right_bumper;
        if (rbNow && !rbPrev) {
            gateToggleAllTheWayUp = !gateToggleAllTheWayUp;

            if (gateToggleAllTheWayUp) gateAllTheWayUp();
            else gateOnePosition();
        }
        rbPrev = rbNow;
    }

    // =========================================================
    // Turret control and tracking
    // - Limelight TX aim when tag==20, else odom aim
    // - FlywheelASG target set here (like auto)
    // - no snap on start; idle hold until shooting mode enabled
    // =========================================================
    private void handleTurretTrackingAndControl(Pose pose) {

        final double TURRET_OFFSET_DEG = 180.0; // backwards is 0
        final double TURRET_MIN_DEG = -160.0;
        final double TURRET_MAX_DEG = 160;
        final long MANUAL_HOLD_MS = 4000;

        long now = System.currentTimeMillis();

        // Toggle shooting (gamepad2 cross)
        boolean shootToggleBtn = gamepad2.cross;
        if (shootToggleBtn && !turretLastShootToggleBtn) {
            turretShootingActive = !turretShootingActive;

            if (turretShootingActive) {
                turretIdleHoldEnabled = false;
            } else {
                flywheelASG.setTargetVelocity(0.0);
                turretIdleHoldEnabled = true;
                turretIdleHoldAngleDeg = turret.getCurrentAngle();
                turret.setAngle(turretIdleHoldAngleDeg);
            }
        }
        turretLastShootToggleBtn = shootToggleBtn;

        // Odom geometry
        double botX = pose.getX();
        double botY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        double dx = TARGET_X - botX;
        double dy = TARGET_Y - botY;

        double distanceToTarget = Math.hypot(dx, dy);

        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double angleToTargetRobotDeg = normalize180(angleToTargetFieldDeg - robotHeadingDeg);
        double desiredTurretDeg = normalize180(angleToTargetRobotDeg + TURRET_OFFSET_DEG);

        double turretCurrentDeg = turret.getCurrentAngle();
        double safeCmdDeg = wrapIntoTurretWindow(
                desiredTurretDeg,
                turretCurrentDeg,
                TURRET_MIN_DEG,
                TURRET_MAX_DEG
        );

        // Manual turret adjust (pauses auto)
        boolean manualInput = false;

        if (gamepad2.dpad_left && (gamepad2.left_trigger > 0.5)) {
            double desired = turret.getTargetAngle() - 10;
            turret.setAngle(wrapIntoTurretWindow(desired, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG));
            manualInput = true;
        } else if (gamepad2.dpad_left) {
            double desired = turret.getTargetAngle() - 1.0;
            turret.setAngle(wrapIntoTurretWindow(desired, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG));
            manualInput = true;
        }

        if (gamepad2.dpad_right && (gamepad2.left_trigger > 0.5)) {
            double desired = turret.getTargetAngle() + 10;
            turret.setAngle(wrapIntoTurretWindow(desired, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG));
            manualInput = true;
        } else if (gamepad2.dpad_right) {
            double desired = turret.getTargetAngle() + 1.0;
            turret.setAngle(wrapIntoTurretWindow(desired, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG));
            manualInput = true;
        }

        if (manualInput) {
            turretManualOverride = true;
            turretLastManualInputMs = now;

            if (turretIdleHoldEnabled) {
                turretIdleHoldAngleDeg = turret.getTargetAngle();
            }
        }

        boolean shareNow = gamepad2.share;
        if (shareNow && !sharePrev) {
            turret.stop();
            turret.zeroTurret();
            turretManualOverride = true;
            turretLastManualInputMs = System.currentTimeMillis();
            gamepad2.rumble(500);

            turretIdleHoldEnabled = true;
            turretIdleHoldAngleDeg = turret.getCurrentAngle();
            turret.setAngle(turretIdleHoldAngleDeg);
        }
        sharePrev = shareNow;

        if (turretManualOverride && (now - turretLastManualInputMs) > MANUAL_HOLD_MS) {
            turretManualOverride = false;
        }

        // =========================
        // Aim + Flywheel command
        // =========================
        if (turretShootingActive) {

            double rpmCmd = computeFlywheelRpmCmd(pose);

            double radPerSec = rpmToRadPerSec(rpmCmd);
            flywheelASG.setTargetVelocity(radPerSec);

            lastCmdRPM = rpmCmd;
            lastCmdRadPerSec = radPerSec;

            // Manual “force turret to 0”
            if (gamepad2.right_trigger > 0.5) {
                turret.setAngle(0);
            } else if (!turretManualOverride) {
                boolean didTxAim = tryLimelightTxAim(pose, TURRET_MIN_DEG, TURRET_MAX_DEG);
                if (!didTxAim) {
                    double desiredOdom = safeCmdDeg + turretOffset;
                    double safeOdom = wrapIntoTurretWindow(
                            desiredOdom,
                            turret.getCurrentAngle(),
                            TURRET_MIN_DEG,
                            TURRET_MAX_DEG
                    );
                    turret.setAngle(safeOdom);
                }
            }

        } else {
            flywheelASG.setTargetVelocity(0.0);
            lastCmdRPM = 0.0;
            lastCmdRadPerSec = 0.0;

            if (!turretManualOverride) {
                if (turretIdleHoldEnabled) {
                    if (!Double.isFinite(turretIdleHoldAngleDeg)) {
                        turretIdleHoldAngleDeg = turretCurrentDeg;
                    }
                    turret.setAngle(wrapIntoTurretWindow(
                            turretIdleHoldAngleDeg,
                            turret.getCurrentAngle(),
                            TURRET_MIN_DEG,
                            TURRET_MAX_DEG
                    ));
                }
            }
        }

        boolean outOfRange = (desiredTurretDeg < -160.0) || (desiredTurretDeg > 160.0);

        if (turretShootingActive && !turretManualOverride && outOfRange) {
            gamepad1.rumble(120);
            gamepad2.rumble(120);
        }

        // Extra telemetry for debugging
        telemetry.addData("DistToTarget(hypot)", String.format(Locale.US, "%.2f", distanceToTarget));
        telemetry.addData("SafeCmdDeg(odom)", String.format(Locale.US, "%.2f", safeCmdDeg));
        telemetry.addData("TurretCurDeg", String.format(Locale.US, "%.2f", turretCurrentDeg));
        telemetry.addData("TurretTgtDeg", String.format(Locale.US, "%.2f", turret.getTargetAngle()));
        telemetry.addData("ManualOverride", turretManualOverride);
    }

    /**
     * Limelight TX aim: only engages if fiducial ID == 20.
     * Holds last good TX for up to LL_TX_HOLD_MS to avoid NaN jitter.
     * Returns true if it issued a turret command this frame using TX.
     */
    private boolean tryLimelightTxAim(Pose pose, double turretMinDeg, double turretMaxDeg) {
        final int TAG_ID_FOR_TX_AIM = 20;

        final double TX_SIGN = +1.0;      // flip if reversed
        final double TX_KP = 0.6;         // deg command per deg tx
        final double MAX_STEP_DEG = 8; // clamp per loop

        llUsingTxAim = false;
        llLastSeenId = -1;
        llLastTxDeg = Double.NaN;

        if (limelight3A == null) return false;

        long now = System.currentTimeMillis();

        // Update robot orientation for Limelight solve
        limelight3A.updateRobotOrientation(Math.toDegrees(pose.getHeading()));
        LLResult result = limelight3A.getLatestResult();

        // If the LL output is temporarily invalid/missing, HOLD turret (no odom fallback) briefly
        if (result == null || !result.isValid()
                || result.getFiducialResults() == null
                || result.getFiducialResults().isEmpty()) {

            if (Double.isFinite(llLastGoodTxDeg) && (now - llLastGoodTxTimeMs) <= LL_TX_HOLD_MS) {
                llUsingTxAim = true;
                llLastTxDeg = llLastGoodTxDeg;

                // actually command something: hold current target
                double hold = wrapIntoTurretWindow(
                        turret.getTargetAngle(),
                        turret.getCurrentAngle(),
                        turretMinDeg,
                        turretMaxDeg
                );
                turret.setAngle(hold);
                return true;
            }
            return false;
        }

        int id = result.getFiducialResults().get(0).getFiducialId();
        llLastSeenId = id;

        if (id != TAG_ID_FOR_TX_AIM) return false;

        LLResult aimRes = limelight3A.getLatestResult();
        if (aimRes == null || !aimRes.isValid()) {
            if (Double.isFinite(llLastGoodTxDeg) && (now - llLastGoodTxTimeMs) <= LL_TX_HOLD_MS) {
                llUsingTxAim = true;
                llLastTxDeg = llLastGoodTxDeg;
                return true;
            }
            return false;
        }

        double txDeg = aimRes.getTx();

        // Save good TX; if TX is NaN, HOLD turret briefly (no odom fallback)
        if (Double.isFinite(txDeg)) {
            llLastGoodTxDeg = txDeg;
            llLastGoodTxTimeMs = now;
            llLastTxDeg = txDeg;
        } else {
            if (Double.isFinite(llLastGoodTxDeg) && (now - llLastGoodTxTimeMs) <= LL_TX_HOLD_MS) {
                llUsingTxAim = true;
                llLastTxDeg = llLastGoodTxDeg;
                return true;
            }
            return false;
        }

        // Normal TX aim command
        double turretCurrentDeg = turret.getCurrentAngle();

        final double TX_SETPOINT_DEG = turretOffset; // Option 1: aim for tx == turretOffset

        double txErr = txDeg - TX_SETPOINT_DEG;

        double delta = -TX_SIGN * TX_KP * txErr;
        delta = Range.clip(delta, -MAX_STEP_DEG, +MAX_STEP_DEG);

// Integrate off TARGET (not current) to avoid measurement noise feeding command directly
        double desired = turret.getTargetAngle() + delta;

        if (!Double.isFinite(desired)) return false;

        double safe = wrapIntoTurretWindow(
                desired,
                turret.getCurrentAngle(),
                turretMinDeg,
                turretMaxDeg
        );

        if (!Double.isFinite(safe)) return false;

        turret.setAngle(safe);
        llUsingTxAim = true;
        return true;
    }

    // =========================================================
    // Flywheel mode select + auto enable toggle (gamepad2)
    // =========================================================
    private void handleFlywheelModeSelect() {

        boolean optNow = gamepad2.options;
        if (optNow && !gp2OptionsPrev) {
            flywheelAutoEnabled = !flywheelAutoEnabled;
            gamepad2.rumble(150);
        }
        gp2OptionsPrev = optNow;

        boolean downNow = gamepad2.dpad_down;
        if (downNow && !gp2DpadDownPrev) {
            flywheelMode = FlywheelMode.CLOSE;
            gamepad2.rumble(80);
            turretOffset = 3;
        }
        gp2DpadDownPrev = downNow;

        boolean upNow = gamepad2.dpad_up;
        if (upNow && !gp2DpadUpPrev) {
            flywheelMode = FlywheelMode.FAR;
            gamepad2.rumble(80);
            turretOffset = 1.5;
        }
        gp2DpadUpPrev = upNow;
    }

    // =========================================================
    // Distance getters (Limelight)
    // =========================================================
    private double getDistanceCamPoseZInches() {
        if (limelight3A == null) return Double.NaN;

        LLResult res = limelight3A.getLatestResult();
        if (res == null || !res.isValid() || res.getFiducialResults() == null || res.getFiducialResults().isEmpty()) {
            return Double.NaN;
        }

        double zMeters = res.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().z;
        if (!Double.isFinite(zMeters)) return Double.NaN;

        return zMeters * METERS_TO_INCHES;
    }

    private double getDistanceAngleMethodInches() {
        if (limelight3A == null) return Double.NaN;

        LLResult res = limelight3A.getLatestResult();
        if (res == null || !res.isValid()) return Double.NaN;

        double tyDeg = res.getTy();
        if (!Double.isFinite(tyDeg)) return Double.NaN;

        double totalDeg = CAM_PITCH_DEG + tyDeg;
        double totalRad = Math.toRadians(totalDeg);

        double denom = Math.tan(totalRad);
        if (!Double.isFinite(denom) || Math.abs(denom) < 1e-6) return Double.NaN;

        double meters = (TARGET_HEIGHT_M - CAM_HEIGHT_M) / denom;
        if (!Double.isFinite(meters)) return Double.NaN;

        return meters * METERS_TO_INCHES;
    }

    // =========================================================
    // Close-zone polynomial placeholder (YOU FILL THIS IN)
    // distIn is inches; return RPM
    // =========================================================
    // =========================================================
// Close-zone RPM from ODOM distance (inches)
// Piecewise:
//  - <= 70 in: flat 2900
//  - >= 72.5 in: cubic regression
//  - >= 95 in: clamp 3120
// =========================================================
    private double closeZoneRpmFromOdom(double distIn) {

        // Below dataset: hold lowest (flat) RPM
        if (distIn <= CLOSE_FLAT_END_IN) return CLOSE_FLAT_RPM;

        // Above dataset max: clamp to max RPM
        if (distIn >= CLOSE_MAX_DIST_IN) return CLOSE_MAX_RPM;

        // Small gap 70..72.5: keep flat (simplest + safest)
        if (distIn < CLOSE_CUBIC_START_IN) return CLOSE_FLAT_RPM;

        // Cubic regression (from your screenshot)
        double x = distIn;
        double rpm =
                0.0162338 * x * x * x
                        - 3.9026 * x * x
                        + 319.58117 * x
                        - 5937.69481;

        if (!Double.isFinite(rpm)) return CLOSE_FLAT_RPM;

        // Final safety clamp: do not exceed your tuned bounds
        return Range.clip(rpm, CLOSE_FLAT_RPM, CLOSE_MAX_RPM);
    }

    // =========================================================
    // Choose distance for auto (cam Z priority, odom fallback)
    // =========================================================
    private double chooseAutoDistanceInches(Pose pose) {
        double cam = getDistanceCamPoseZInches();
        if (Double.isFinite(cam)) return cam;

        double odom = computeDistanceToTargetInches(pose);
        if (Double.isFinite(odom)) return odom;

        return Double.NaN;
    }

    // =========================================================
    // Compute RPM command given mode + auto enabled
    // =========================================================
    private double computeFlywheelRpmCmd(Pose pose) {

        // FAR mode is always fixed RPM (even if auto enabled)
        if (flywheelMode == FlywheelMode.FAR) {
            distAutoUsedIn = Double.NaN;
            return FAR_FIXED_RPM;
        }

        // CLOSE mode (manual override when auto disabled)
        if (!flywheelAutoEnabled) {
            distAutoUsedIn = Double.NaN;
            return flywheelTuneRPM;
        }

        // CLOSE mode (AUTO): ODOM DISTANCE ONLY
        double dIn = computeDistanceToTargetInches(pose);
        distAutoUsedIn = dIn;

        if (!Double.isFinite(dIn)) {
            return flywheelTuneRPM;
        }

        // Optional: below dataset start, hold lowest
        if (dIn <= CLOSE_MIN_DIST_IN) return CLOSE_FLAT_RPM;

        return closeZoneRpmFromOdom(dIn);
    }

    /**
     * Maps desiredDeg to an equivalent angle within [minDeg, maxDeg] by trying desired +/- 360*k.
     * If multiple equivalents are valid, chooses the one closest to referenceDeg.
     */
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

        if (Double.isNaN(best)) best = Range.clip(desiredDeg, minDeg, maxDeg);
        return best;
    }

    // =========================================================
    // Drive logic
    // =========================================================
    private void drive(double robotHeadingDeg) {
        double trigger = Range.clip(1 - gamepad1.right_trigger, 0.2, 1);

        if (!(gamepad1.left_trigger > 0.5)) {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y * trigger,
                    gamepad1.left_stick_x * trigger,
                    -gamepad1.right_stick_x * trigger,
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

        if (gamepad1.circle) {
            follower.setPose(new Pose(24.4, 126, Math.toRadians(142)));
            gamepad1.rumble(500);
        }
    }

    private double normalize180(double a) {
        return ((a + 180) % 360 + 360) % 360 - 180;
    }

    // =========================================================
    // Gate helpers
    // =========================================================
    private void gateAllTheWayUp() {
        basePlate.gateUp();
    }

    private void gateOnePosition() {
        basePlate.gateHoldBall1();
    }

    private void gateDown() {
        basePlate.gateHoldBall1();
    }

    // =========================================================
    // Distance helper (odometry)
    // =========================================================
    private double computeDistanceToTargetInches(Pose pose) {
        double dx = TARGET_X - pose.getX();
        double dy = TARGET_Y - pose.getY();
        return Math.hypot(dx, dy);
    }

    // =========================================================
    // Flywheel unit conversion (RPM -> rad/s)
    // =========================================================
    private double rpmToRadPerSec(double rpm) {
        return rpm * (2.0 * Math.PI / 60.0);
    }

    // =========================================================
    // Flywheel manual tuning (gamepad1 dpad up/down)
    // =========================================================
    private void handleDriverFlywheelTuning() {
        boolean upNow = gamepad1.dpad_up;
        if (upNow && !gp1DpadUpPrev) {
            flywheelTuneRPM = Range.clip(flywheelTuneRPM + FLYWHEEL_RPM_STEP, FLYWHEEL_RPM_MIN, FLYWHEEL_RPM_MAX);
            gamepad1.rumble(80);
        }
        gp1DpadUpPrev = upNow;

        boolean downNow = gamepad1.dpad_down;
        if (downNow && !gp1DpadDownPrev) {
            flywheelTuneRPM = Range.clip(flywheelTuneRPM - FLYWHEEL_RPM_STEP, FLYWHEEL_RPM_MIN, FLYWHEEL_RPM_MAX);
            gamepad1.rumble(80);
        }
        gp1DpadDownPrev = downNow;
    }
}