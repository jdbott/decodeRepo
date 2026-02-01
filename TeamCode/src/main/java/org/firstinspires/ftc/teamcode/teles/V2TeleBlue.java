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

import org.firstinspires.ftc.teamcode.hardwareClasses.BasePlate;
import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;
import org.firstinspires.ftc.teamcode.hardwareClasses.Gantry;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.ArrayList;
import java.util.Locale;

@TeleOp(name = "V2 TELEOP BLUE")
public class V2TeleBlue extends LinearOpMode {

    // --- Subsystems / hardware ---
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlate basePlate;

    // Flywheel (NEW: FlywheelASG like auto)
    private FlywheelASG flywheelASG;

    // Drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Turret object (kept as a field so we can init once)
    private final Turret turret = new Turret();

    // Edge memory for basePlate LB trigger
    private boolean lbPrev = false;

    // =========================
    // Limelight
    // =========================
    private Limelight3A limelight3A;
    private int llLastSeenId = -1;
    private double llLastTxDeg = Double.NaN;
    private boolean llUsingTxAim = false;

    // =========================
    // Intake FSM
    // =========================
    private enum IntakeState {
        OFF,
        ON,
        SHUTDOWN_WAIT // gate down now; wait 0.4s then stop intake motor
    }

    private IntakeState intakeState = IntakeState.OFF;
    private boolean intakeToggleActive = false;
    private boolean lastSquareButton = false;
    private long intakeShutdownStartMs = 0;

    private static final long INTAKE_SHUTDOWN_DELAY_MS = 400;

    // =========================
    // Gate toggle (RB)
    // =========================
    private boolean rbPrev = false;
    private boolean gateToggleAllTheWayUp = true; // true = all-the-way-up, false = "one position"

    // =========================
    // Turret state fields
    // =========================
    private boolean turretShootingActive = false;
    private boolean turretLastShootToggleBtn = false;

    private boolean turretManualOverride = false;
    private long turretLastManualInputMs = 0;

    private boolean sharePrev = false;

    // =========================
    // Target (for odom aim + distance)
    // =========================
    private static final double TARGET_X = 0.0;
    private static final double TARGET_Y = 148.0;

    private double lastDistanceToTargetIn = 0.0;

    // =========================
    // Flywheel tuning (RPM on gamepad1)
    // =========================
    private double flywheelTuneRPM = 3400;

    private boolean gp1DpadUpPrev = false;
    private boolean gp1DpadDownPrev = false;
    private boolean gp1SquarePrev = false;
    private boolean gp1TrianglePrev = false;

    private final ArrayList<Double> logDist = new ArrayList<>();
    private final ArrayList<Double> logRPM = new ArrayList<>();

    private double lastLoggedDist = Double.NaN;
    private double lastLoggedRPM = Double.NaN;

    private int fittedDegree = -1;
    private double[] fittedCoeffs = null; // a0 + a1*x + ...

    private static final double FLYWHEEL_RPM_STEP = 20.0;
    private static final double FLYWHEEL_RPM_MIN = 0.0;
    private static final double FLYWHEEL_RPM_MAX = 6000.0;

    private static final int POLY_MAX_DEGREE = 4;

    private double minLoggedDist = Double.NaN;
    private double maxLoggedDist = Double.NaN;

    // For telemetry: what we commanded this frame
    private double lastCmdRPM = 0.0;
    private double lastCmdRadPerSec = 0.0;

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
        basePlate = new BasePlate(hardwareMap);

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

        while (opModeIsActive()) {

            // optional override you had
            if (gamepad2.dpad_down) {
                basePlate.gateBackFullShoot();
            }

            follower.update();

            Pose pose = follower.getPose();

            // -----------------------------
            // Drive
            // -----------------------------
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());
            drive(robotHeadingDeg);

            // -----------------------------
            // Distance + tuning
            // -----------------------------
            lastDistanceToTargetIn = computeDistanceToTargetInches(pose);
            handleDriverFlywheelTuningAndPolyFit();

            // -----------------------------
            // Intake toggle + FSM
            // -----------------------------
            if (gamepad2.dpad_up) {
                intake.intakeOut();
            }
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

            telemetry.addData("LL_UsingTxAim", llUsingTxAim);
            telemetry.addData("LL_LastId", llLastSeenId);
            telemetry.addData("LL_TxDeg", Double.isNaN(llLastTxDeg) ? "NaN" : String.format(Locale.US, "%.2f", llLastTxDeg));

            telemetry.addData("DistToTarget(in)", String.format(Locale.US, "%.2f", lastDistanceToTargetIn));
            telemetry.addData("TuneRPM(gamepad1)", String.format(Locale.US, "%.0f", flywheelTuneRPM));

            telemetry.addData("FlywheelCmdRPM", String.format(Locale.US, "%.0f", lastCmdRPM));
            telemetry.addData("FlywheelCmdRadS", String.format(Locale.US, "%.2f", lastCmdRadPerSec));

            telemetry.addData("LoggedPoints", logDist.size());
            if (!Double.isNaN(lastLoggedDist)) {
                telemetry.addData("LastLog", String.format(Locale.US, "d=%.2f, rpm=%.0f", lastLoggedDist, lastLoggedRPM));
            }
            if (fittedCoeffs != null) {
                telemetry.addData("PolyFit", "deg=" + fittedDegree);
                telemetry.addData("PolyEq", polynomialToString(fittedCoeffs));
            } else {
                telemetry.addData("PolyFit", "none");
            }

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
                intake.intakeIn();
                gateAllTheWayUp();
                break;

            case SHUTDOWN_WAIT:
                intake.intakeStop();
                gateDown();

                if (System.currentTimeMillis() - intakeShutdownStartMs >= INTAKE_SHUTDOWN_DELAY_MS) {
                    gateDown();
                    intakeState = IntakeState.OFF;
                    basePlate.prepShootOnly();
                } else {
                    intake.intakeIn();
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
    // =========================================================
    private void handleTurretTrackingAndControl(Pose pose) {

        final double TURRET_OFFSET_DEG = 180.0; // backwards is 0
        final double TURRET_MIN_DEG = -160.0;
        final double TURRET_MAX_DEG = 200.0;
        final long MANUAL_HOLD_MS = 4000;

        long now = System.currentTimeMillis();

        // Toggle shooting (gamepad2 cross)
        boolean shootToggleBtn = gamepad2.cross;
        if (shootToggleBtn && !turretLastShootToggleBtn) {
            turretShootingActive = !turretShootingActive;

            if (!turretShootingActive) {
                // stop flywheel immediately
                flywheelASG.setTargetVelocity(0.0);
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
        }

        boolean shareNow = gamepad2.share;
        if (shareNow && !sharePrev) {
            turret.stop();
            turret.zeroTurret();
            turretManualOverride = true;
            turretLastManualInputMs = System.currentTimeMillis();
            gamepad2.rumble(500);
        }
        sharePrev = shareNow;

        if (turretManualOverride && (now - turretLastManualInputMs) > MANUAL_HOLD_MS) {
            turretManualOverride = false;
        }

        // =========================
        // Aim + Flywheel command
        // =========================
        if (turretShootingActive) {

            // default: manual tuning RPM
            double rpmCmd = flywheelTuneRPM;

            // Optional: auto RPM from fitted polynomial
            /*
            if (fittedCoeffs != null && !Double.isNaN(minLoggedDist) && !Double.isNaN(maxLoggedDist)) {
                double dClamped = Range.clip(distanceToTarget, minLoggedDist, maxLoggedDist);
                rpmCmd = evalPolynomial(fittedCoeffs, dClamped);
                rpmCmd = Range.clip(rpmCmd, FLYWHEEL_RPM_MIN, FLYWHEEL_RPM_MAX);
            }
            */

            // FlywheelASG uses rad/s (like auto)
            double radPerSec = rpmToRadPerSec(rpmCmd);
            flywheelASG.setTargetVelocity(radPerSec);

            lastCmdRPM = rpmCmd;
            lastCmdRadPerSec = radPerSec;

            // Manual “force turret to 0”
            if (gamepad2.right_trigger > 0.5) {
                turret.setAngle(0);
            } else if (!turretManualOverride) {
                boolean didTxAim = tryLimelightTxAim(pose, TURRET_MIN_DEG, TURRET_MAX_DEG);
                if (!didTxAim) turret.setAngle(safeCmdDeg);
            }

        } else {
            // Not shooting: stop flywheel; park turret unless manual override
            flywheelASG.setTargetVelocity(0.0);
            lastCmdRPM = 0.0;
            lastCmdRadPerSec = 0.0;

            if (!turretManualOverride) {
                double park = wrapIntoTurretWindow(135, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
                turret.setAngle(park);
            }
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
     * Returns true if it issued a turret command this frame using TX.
     */
    private boolean tryLimelightTxAim(Pose pose, double turretMinDeg, double turretMaxDeg) {
        final int TAG_ID_FOR_TX_AIM = 20;

        final double TX_SIGN = +1.0;      // flip if reversed
        final double TX_KP = 0.7;         // deg command per deg tx
        final double MAX_STEP_DEG = 10.0; // clamp per loop

        llUsingTxAim = false;
        llLastSeenId = -1;
        llLastTxDeg = Double.NaN;

        if (limelight3A == null) return false;

        LLResult result = limelight3A.getLatestResult();
        if (result == null || !result.isValid()
                || result.getFiducialResults() == null
                || result.getFiducialResults().isEmpty()) {
            return false;
        }

        int id = result.getFiducialResults().get(0).getFiducialId();
        llLastSeenId = id;
        if (id != TAG_ID_FOR_TX_AIM) return false;

        limelight3A.updateRobotOrientation(Math.toDegrees(pose.getHeading()));

        LLResult aimRes = limelight3A.getLatestResult();
        if (aimRes == null || !aimRes.isValid()) return false;

        double txDeg = aimRes.getTx();
        llLastTxDeg = txDeg;

        double turretCurrentDeg = turret.getCurrentAngle();
        double delta = -TX_SIGN * TX_KP * txDeg;
        delta = Range.clip(delta, -MAX_STEP_DEG, +MAX_STEP_DEG);

        double desired = turretCurrentDeg + delta;

        double safe = wrapIntoTurretWindow(
                desired,
                turret.getTargetAngle(),
                turretMinDeg,
                turretMaxDeg
        );

        turret.setAngle(safe);
        llUsingTxAim = true;
        return true;
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

    // -----------------------------
    // Drive logic
    // -----------------------------
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
    // Distance helper
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
    // Tuning + poly fit (unchanged)
    // =========================================================
    private void handleDriverFlywheelTuningAndPolyFit() {
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

        boolean squareNow = gamepad1.square;
        if (squareNow && !gp1SquarePrev) {
            logDist.add(lastDistanceToTargetIn);
            logRPM.add(flywheelTuneRPM);
            lastLoggedDist = lastDistanceToTargetIn;
            lastLoggedRPM = flywheelTuneRPM;

            if (Double.isNaN(minLoggedDist) || lastDistanceToTargetIn < minLoggedDist) minLoggedDist = lastDistanceToTargetIn;
            if (Double.isNaN(maxLoggedDist) || lastDistanceToTargetIn > maxLoggedDist) maxLoggedDist = lastDistanceToTargetIn;

            gamepad1.rumble(200);
        }
        gp1SquarePrev = squareNow;

        boolean triNow = gamepad1.triangle;
        if (triNow && !gp1TrianglePrev) {
            if (logDist.size() >= 2) {
                FitResult fit = fitBestPolynomialAIC(logDist, logRPM, POLY_MAX_DEGREE);
                fittedDegree = fit.degree;
                fittedCoeffs = fit.coeffs;
                gamepad1.rumble(400);
            } else {
                fittedDegree = -1;
                fittedCoeffs = null;
                gamepad1.rumble(120);
            }
        }
        gp1TrianglePrev = triNow;
    }

    private static class FitResult {
        int degree;
        double[] coeffs;
        double aic;
        double sse;

        FitResult(int degree, double[] coeffs, double aic, double sse) {
            this.degree = degree;
            this.coeffs = coeffs;
            this.aic = aic;
            this.sse = sse;
        }
    }

    private FitResult fitBestPolynomialAIC(ArrayList<Double> x, ArrayList<Double> y, int maxDeg) {
        int n = x.size();
        int upper = Math.min(maxDeg, n - 1);
        FitResult best = null;

        for (int deg = 1; deg <= upper; deg++) {
            double[] coeffs = leastSquaresPolyFit(x, y, deg);
            if (coeffs == null) continue;

            double sse = 0.0;
            for (int i = 0; i < n; i++) {
                double yi = evalPolynomial(coeffs, x.get(i));
                double err = (y.get(i) - yi);
                sse += err * err;
            }

            double ssePer = Math.max(sse / n, 1e-9);
            int k = deg + 1;
            double aic = n * Math.log(ssePer) + 2.0 * k;

            FitResult r = new FitResult(deg, coeffs, aic, sse);
            if (best == null || r.aic < best.aic) best = r;
        }

        return best;
    }

    private double[] leastSquaresPolyFit(ArrayList<Double> x, ArrayList<Double> y, int degree) {
        int n = x.size();
        int m = degree + 1;

        double[][] ata = new double[m][m];
        double[] aty = new double[m];

        for (int i = 0; i < n; i++) {
            double xi = x.get(i);
            double yi = y.get(i);

            double[] pow = new double[2 * degree + 1];
            pow[0] = 1.0;
            for (int p = 1; p < pow.length; p++) pow[p] = pow[p - 1] * xi;

            for (int row = 0; row < m; row++) {
                for (int col = 0; col < m; col++) {
                    ata[row][col] += pow[row + col];
                }
                aty[row] += yi * pow[row];
            }
        }

        return solveLinearSystem(ata, aty);
    }

    private double[] solveLinearSystem(double[][] M, double[] b) {
        int n = b.length;

        double[][] A = new double[n][n + 1];
        for (int r = 0; r < n; r++) {
            System.arraycopy(M[r], 0, A[r], 0, n);
            A[r][n] = b[r];
        }

        for (int p = 0; p < n; p++) {
            int maxRow = p;
            double maxVal = Math.abs(A[p][p]);
            for (int r = p + 1; r < n; r++) {
                double v = Math.abs(A[r][p]);
                if (v > maxVal) {
                    maxVal = v;
                    maxRow = r;
                }
            }

            if (maxVal < 1e-10) return null;

            if (maxRow != p) {
                double[] tmp = A[p];
                A[p] = A[maxRow];
                A[maxRow] = tmp;
            }

            double pivot = A[p][p];
            for (int c = p; c < n + 1; c++) A[p][c] /= pivot;

            for (int r = 0; r < n; r++) {
                if (r == p) continue;
                double factor = A[r][p];
                for (int c = p; c < n + 1; c++) {
                    A[r][c] -= factor * A[p][c];
                }
            }
        }

        double[] x = new double[n];
        for (int i = 0; i < n; i++) x[i] = A[i][n];
        return x;
    }

    private double evalPolynomial(double[] coeffs, double x) {
        double y = 0.0;
        for (int i = coeffs.length - 1; i >= 0; i--) {
            y = y * x + coeffs[i];
        }
        return y;
    }

    private String polynomialToString(double[] coeffs) {
        StringBuilder sb = new StringBuilder("rpm = ");
        for (int i = 0; i < coeffs.length; i++) {
            double a = coeffs[i];
            if (i == 0) {
                sb.append(String.format(Locale.US, "%.6f", a));
            } else {
                sb.append(a >= 0 ? " + " : " - ");
                sb.append(String.format(Locale.US, "%.6f", Math.abs(a)));
                sb.append("*d");
                if (i >= 2) sb.append("^").append(i);
            }
        }
        return sb.toString();
    }
}
