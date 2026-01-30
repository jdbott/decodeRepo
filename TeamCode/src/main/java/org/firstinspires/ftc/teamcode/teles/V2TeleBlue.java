package org.firstinspires.ftc.teamcode.teles;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardwareClasses.BasePlateFast;
import org.firstinspires.ftc.teamcode.hardwareClasses.Gantry;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.ShooterV2;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.ArrayList;
import java.util.Locale;
/* =========================
   >>> ADDED IMPORTS END <<<
   ========================= */

@TeleOp(name = "V2 TELEOP BLUE")
public class V2TeleBlue extends LinearOpMode {

    // --- Subsystems / hardware ---
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlateFast basePlate;
    private ShooterV2 shooter;

    // Drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Turret object (kept as a field so we can init once)
    private final Turret turret = new Turret();

    // Edge memory for basePlate LB trigger
    private boolean lbPrev = false;

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

    // ---- Turret FF (heading-rate) ----
    private double lastHeadingDeg = 0.0;
    private long lastHeadingMs = 0;

    private boolean sharePrev = false;

    /* ============================================
       >>> ADDED: DISTANCE + FLYWHEEL TUNING START <<<
       ============================================ */

    // --- Target location (same as turret logic) ---
    private static final double TARGET_X = 0.0;
    private static final double TARGET_Y = 148.0;

    // Latest distance (inches) so we can display/log from anywhere.
    private double lastDistanceToTargetIn = 0.0;

    // Manual tuning RPM (driver-controlled). Start at something reasonable.
    private double flywheelTuneRPM = 2950;

    // Driver edge-detect for tuning controls
    private boolean gp1DpadUpPrev = false;
    private boolean gp1DpadDownPrev = false;
    private boolean gp1SquarePrev = false;
    private boolean gp1TrianglePrev = false;

    // Logged data points: x = distance (in), y = RPM
    private final ArrayList<Double> logDist = new ArrayList<>();
    private final ArrayList<Double> logRPM = new ArrayList<>();

    // Last logged point (for quick feedback)
    private double lastLoggedDist = Double.NaN;
    private double lastLoggedRPM = Double.NaN;

    // Polynomial fit storage
    private int fittedDegree = -1;
    private double[] fittedCoeffs = null; // a0 + a1*x + a2*x^2 + ...

    // Limits / controls
    private static final double FLYWHEEL_RPM_STEP = 20.0;
    private static final double FLYWHEEL_RPM_MIN = 0.0;
    private static final double FLYWHEEL_RPM_MAX = 6000.0;

    // Degrees to consider when fitting (kept conservative for on-robot compute)
    private static final int POLY_MAX_DEGREE = 4;

    private double minLoggedDist = Double.NaN;
    private double maxLoggedDist = Double.NaN;

    /* ==========================================
       >>> ADDED: DISTANCE + FLYWHEEL TUNING END <<<
       ========================================== */

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
        // Hardware map
        // -----------------------------
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Drivetrain directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Subsystems
        intake = new Intake(hardwareMap);
        gantry = new Gantry(hardwareMap);
        basePlate = new BasePlateFast(hardwareMap);

        shooter = new ShooterV2();
        shooter.init(hardwareMap,
                "shooter2",
                "shooter1",
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE
        );

        // Safe defaults
        intake.intakeStop();
        gantry.moveGantryToPos("back");
        basePlate.rampBack();
        basePlate.frontPopperDown();
        basePlate.middlePopperDown();
        basePlate.cancelShootAndReset();

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
        }

        waitForStart();
        resetRuntime();

        // Seed heading-rate estimator
        Pose startPose = follower.getPose();
        lastHeadingDeg = Math.toDegrees(startPose.getHeading());
        lastHeadingMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (gamepad2.dpad_down) {
                basePlate.gateBackFullShoot();
            }

            follower.update();
            shooter.update();

            Pose pose = follower.getPose();

            // -----------------------------
            // Drive
            // -----------------------------
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());
            drive(robotHeadingDeg);

            /* ============================================
               >>> ADDED: UPDATE DISTANCE + HANDLE TUNING <<<
               ============================================ */
            lastDistanceToTargetIn = computeDistanceToTargetInches(pose);
            handleDriverFlywheelTuningAndPolyFit(); // uses gamepad1
            /* ==========================================
               >>> ADDED: UPDATE DISTANCE + HANDLE TUNING END
               ========================================== */

            // -----------------------------
            // Intake toggle + FSM
            // -----------------------------
            if (gamepad2.dpad_up) {
                intake.intakeOut();
            }
            handleIntakeToggleAndFSM();

            // -----------------------------
            // Gate toggle on RB
            // (Intake FSM overrides this while intake is ON or shutting down)
            // -----------------------------
            handleGateToggle();

            // -----------------------------
            // Turret logic (fixed targeting using actual turret angle)
            // - Cross toggles tracking AND flywheels
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

            basePlate.update();

            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.addData("IntakeState", intakeState);
            telemetry.addData("GateMode", gateToggleAllTheWayUp ? "ALL_UP" : "ONE_POS");
            telemetry.addData("ShootMode", turretShootingActive);

            /* ============================================
               >>> ADDED: TELEMETRY FOR TUNING + FIT <<<
               ============================================ */
            telemetry.addData("DistToTarget(in)", String.format(Locale.US, "%.2f", lastDistanceToTargetIn));
            telemetry.addData("TuneRPM(gamepad1)", String.format(Locale.US, "%.0f", flywheelTuneRPM));
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
            /* ==========================================
               >>> ADDED: TELEMETRY FOR TUNING + FIT END
               ========================================== */

            telemetry.update();
        }
    }

    // =========================================================
    // Intake toggle + FSM
    // - When toggled ON: intake motor ON + gate ALL THE WAY UP
    // - When toggled OFF: gate DOWN immediately, then 0.4s later intake OFF
    // =========================================================
    private void handleIntakeToggleAndFSM() {
        // Toggle intent (gamepad2 square)
        boolean squareButton = gamepad2.square;
        if (squareButton && !lastSquareButton) {
            intakeToggleActive = !intakeToggleActive;

            if (intakeToggleActive) {
                // Immediately go ON
                intakeState = IntakeState.ON;
                basePlate.cancelShootAndReset(); // one-time on entry to intake mode
            } else {
                // Begin shutdown sequence:
                // gate goes down now; intake stops after delay
                intakeState = IntakeState.SHUTDOWN_WAIT;
                intakeShutdownStartMs = System.currentTimeMillis();
            }
        }
        lastSquareButton = squareButton;

        // FSM execution
        switch (intakeState) {
            case OFF:
                intake.intakeStop();
                break;

            case ON:
                intake.intakeIn();
                gateAllTheWayUp();
                break;

            case SHUTDOWN_WAIT:
                gateDown();

                if (System.currentTimeMillis() - intakeShutdownStartMs >= INTAKE_SHUTDOWN_DELAY_MS) {
                    intake.intakeStop();
                    intakeState = IntakeState.OFF;
                    basePlate.prepShootOnly();
                } else {
                    // Keep intake motor running during the wait
                    intake.intakeIn();
                }
                break;
        }
    }

    // =========================================================
    // Gate toggle (RB)
    // - Toggles between ALL-THE-WAY-UP and ONE-POSITION
    // - Intake FSM overrides gate while ON or shutting down
    // =========================================================
    private void handleGateToggle() {
        if (intakeState == IntakeState.ON || intakeState == IntakeState.SHUTDOWN_WAIT) return;

        boolean rbNow = gamepad2.right_bumper;
        if (rbNow && !rbPrev) {
            gateToggleAllTheWayUp = !gateToggleAllTheWayUp;

            if (gateToggleAllTheWayUp) {
                gateAllTheWayUp();
            } else {
                gateOnePosition();
            }
        }
        rbPrev = rbNow;
    }

    private double getShooterRPMForDistance(double distanceInches) {
        return 3500;
    }

    // =========================================================
    // Turret control and tracking (fixed targeting)
    //
    // Your key detail: "backwards is 0".
    // That means: turretAngle = 0 points backward relative to robot.
    // To convert "angle-to-target in robot frame" to your turret coords, we add +180.
    // =========================================================
    private void handleTurretTrackingAndControl(Pose pose) {
        // =========================
        // Config
        // =========================
        final double TARGET_X = 0;
        final double TARGET_Y = 148;

        // Backwards is 0 => forward is 180
        final double TURRET_OFFSET_DEG = 180.0;

        final double TURRET_MIN_DEG = -160.0;
        final double TURRET_MAX_DEG = 200.0;

        final long MANUAL_HOLD_MS = 4000;

        long now = System.currentTimeMillis();

        // =========================
        // Toggle tracking + flywheels (gamepad2 cross)
        // =========================
        boolean shootToggleBtn = gamepad2.cross;
        if (shootToggleBtn && !turretLastShootToggleBtn) {
            turretShootingActive = !turretShootingActive;

            if (!turretShootingActive) {
                shooter.stop();
            }
        }
        turretLastShootToggleBtn = shootToggleBtn;

        // =========================
        // Pose / geometry
        // =========================
        double botX = pose.getX();
        double botY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        double dx = TARGET_X - botX;
        double dy = TARGET_Y - botY;

        double distanceToTarget = Math.hypot(dx, dy);

        // Field-frame angle from robot -> target
        double angleToTargetFieldDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Convert to robot frame (relative bearing)
        double angleToTargetRobotDeg = normalize180(angleToTargetFieldDeg - robotHeadingDeg);

        // Convert robot-frame bearing to your turret coordinates (0 = backwards)
        double desiredTurretDeg = normalize180(angleToTargetRobotDeg + TURRET_OFFSET_DEG);

        // Use actual turret angle to pick the best wrapped equivalent in limits
        double turretCurrentDeg = turret.getCurrentAngle();

        double safeCmdDeg = wrapIntoTurretWindow(
                desiredTurretDeg,
                turretCurrentDeg,
                TURRET_MIN_DEG,
                TURRET_MAX_DEG
        );

        // =========================
        // Manual turret adjust (pauses auto)
        // =========================
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
            turret.stop();       // optional but recommended
            turret.zeroTurret();
            turretManualOverride = true;          // keep auto-park from instantly fighting you
            turretLastManualInputMs = System.currentTimeMillis();
            gamepad2.rumble(500);
        }
        sharePrev = shareNow;

        if (turretManualOverride && (now - turretLastManualInputMs) > MANUAL_HOLD_MS) {
            turretManualOverride = false;
        }

        // =========================
        // Auto aim + shooter RPM
        // =========================
        if (turretShootingActive) {

            /* =========================================================
               >>> ADDED: RPM COMMAND SOURCE (TUNING vs POLY AUTO) <<<
               ========================================================= */

            // During tuning, we drive the RPM from gamepad1 increments:
            double rpmCmd = flywheelTuneRPM;

            // When you're ready to enable automatic RPM from your polynomial fit,
            // uncomment the block below. It uses the fitted polynomial coefficients
            // (distance in inches -> RPM), and clips to your allowed range.

            /*
            if (fittedCoeffs != null && !Double.isNaN(minLoggedDist) && !Double.isNaN(maxLoggedDist)) {

                // Clamp distance to the region you actually sampled
                double dClamped = Range.clip(distanceToTarget, minLoggedDist, maxLoggedDist);

                rpmCmd = evalPolynomial(fittedCoeffs, dClamped);
                rpmCmd = Range.clip(rpmCmd, FLYWHEEL_RPM_MIN, FLYWHEEL_RPM_MAX);
            }
            */

            shooter.setEnabled(true);
            shooter.setTargetRPM(rpmCmd);

            /* =======================================================
               >>> ADDED: RPM COMMAND SOURCE (TUNING vs POLY AUTO) END
               ======================================================= */

            if ((gamepad2.right_trigger > 0.5)) {
                turret.setAngle(0);
            }
            if (!turretManualOverride && !(gamepad2.right_trigger > 0.5)) {
                turret.setAngle(safeCmdDeg);
            }

        } else {
            // Not tracking: stop shooter; park turret (unless manual override)
            if (!turretManualOverride) {
                double park = wrapIntoTurretWindow(135, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
                turret.setAngle(park);
            }
        }

        turret.update();

        // =========================
        // Telemetry (existing)
        // =========================
        telemetry.addData("DistToTarget", distanceToTarget);
        telemetry.addData("AngleToTargetField", angleToTargetFieldDeg);
        telemetry.addData("AngleToTargetRobot", angleToTargetRobotDeg);
        telemetry.addData("DesiredTurretDeg", desiredTurretDeg);
        telemetry.addData("SafeCmdDeg", safeCmdDeg);
        telemetry.addData("TurretCurDeg", turretCurrentDeg);
        telemetry.addData("TurretTgtDeg", turret.getTargetAngle());
        telemetry.addData("ManualOverride", turretManualOverride);
        telemetry.addData("Flywheels", turretShootingActive ? "ON" : "OFF");
        telemetry.addData("ShooterRPM", shooter.getVelocityRPM());
        telemetry.addData("TargetRPM", shooter.getTargetRPM());
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

        if (Double.isNaN(best)) {
            best = Range.clip(desiredDeg, minDeg, maxDeg);
        }

        return best;
    }

    // -----------------------------
    // Extracted drive logic
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
        a = ((a + 180) % 360 + 360) % 360 - 180;
        return a;
    }

    // =========================================================
    // Gate helpers
    // IMPORTANT: adjust these 3 methods to match your BasePlate API.
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

    /* ============================================================
       >>> ADDED: DISTANCE + TUNING + POLY FIT IMPLEMENTATION START <<<
       ============================================================ */

    private double computeDistanceToTargetInches(Pose pose) {
        double dx = TARGET_X - pose.getX();
        double dy = TARGET_Y - pose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * gamepad1 controls:
     * - dpad_up/down: adjust flywheelTuneRPM by 20
     * - square: log point (distance, RPM)
     * - triangle: fit polynomial, choose best degree, print equation
     */
    private void handleDriverFlywheelTuningAndPolyFit() {
        // --- Adjust RPM by 20 using dpad up/down (edge-triggered) ---
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

        // --- Log a data point (distance, RPM) ---
        boolean squareNow = gamepad1.square;
        if (squareNow && !gp1SquarePrev) {
            logDist.add(lastDistanceToTargetIn);
            logRPM.add(flywheelTuneRPM);
            lastLoggedDist = lastDistanceToTargetIn;
            lastLoggedRPM = flywheelTuneRPM;
            if (Double.isNaN(minLoggedDist) || lastDistanceToTargetIn < minLoggedDist)
                minLoggedDist = lastDistanceToTargetIn;
            if (Double.isNaN(maxLoggedDist) || lastDistanceToTargetIn > maxLoggedDist)
                maxLoggedDist = lastDistanceToTargetIn;
            gamepad1.rumble(200);
        }
        gp1SquarePrev = squareNow;

        // --- Fit polynomial and output equation ---
        boolean triNow = gamepad1.triangle;
        if (triNow && !gp1TrianglePrev) {
            if (logDist.size() >= 2) {
                FitResult fit = fitBestPolynomialAIC(logDist, logRPM, POLY_MAX_DEGREE);
                fittedDegree = fit.degree;
                fittedCoeffs = fit.coeffs;
                gamepad1.rumble(400);
            } else {
                // Not enough points to fit anything meaningful
                fittedDegree = -1;
                fittedCoeffs = null;
                gamepad1.rumble(120);
            }
        }
        gp1TrianglePrev = triNow;
    }

    // ---------- Polynomial fit utilities ----------

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

    /**
     * Chooses best polynomial degree via AIC over degrees 1..min(maxDeg, n-1).
     * - Fits y = a0 + a1 x + ... + ad x^d by least squares (normal equations).
     * - Uses AIC = n*ln(SSE/n) + 2k, k = (d+1) parameters.
     */
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

            // Avoid log(0) if perfect fit (rare); clamp to tiny.
            double ssePer = Math.max(sse / n, 1e-9);
            int k = deg + 1;
            double aic = n * Math.log(ssePer) + 2.0 * k;

            FitResult r = new FitResult(deg, coeffs, aic, sse);
            if (best == null || r.aic < best.aic) best = r;
        }

        return best;
    }

    /**
     * Least squares polynomial fit using normal equations:
     * (A^T A) c = (A^T y), where A[i,j] = x_i^j.
     * Solves with Gaussian elimination with partial pivoting.
     */
    private double[] leastSquaresPolyFit(ArrayList<Double> x, ArrayList<Double> y, int degree) {
        int n = x.size();
        int m = degree + 1;

        double[][] ata = new double[m][m];
        double[] aty = new double[m];

        // Build normal equations
        for (int i = 0; i < n; i++) {
            double xi = x.get(i);
            double yi = y.get(i);

            // Powers of xi up to 2*degree for ata accumulation
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

    /**
     * Solves Mx = b using Gaussian elimination w/ partial pivoting.
     * Returns null if singular/ill-conditioned.
     */
    private double[] solveLinearSystem(double[][] M, double[] b) {
        int n = b.length;

        // Augment matrix
        double[][] A = new double[n][n + 1];
        for (int r = 0; r < n; r++) {
            System.arraycopy(M[r], 0, A[r], 0, n);
            A[r][n] = b[r];
        }

        for (int p = 0; p < n; p++) {
            // Pivot
            int maxRow = p;
            double maxVal = Math.abs(A[p][p]);
            for (int r = p + 1; r < n; r++) {
                double v = Math.abs(A[r][p]);
                if (v > maxVal) {
                    maxVal = v;
                    maxRow = r;
                }
            }

            // Singular?
            if (maxVal < 1e-10) return null;

            // Swap
            if (maxRow != p) {
                double[] tmp = A[p];
                A[p] = A[maxRow];
                A[maxRow] = tmp;
            }

            // Normalize pivot row
            double pivot = A[p][p];
            for (int c = p; c < n + 1; c++) A[p][c] /= pivot;

            // Eliminate
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
        // Horner's method
        double y = 0.0;
        for (int i = coeffs.length - 1; i >= 0; i--) {
            y = y * x + coeffs[i];
        }
        return y;
    }

    private String polynomialToString(double[] coeffs) {
        // y = a0 + a1 x + a2 x^2 + ...
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

    /* ==========================================================
       >>> ADDED: DISTANCE + TUNING + POLY FIT IMPLEMENTATION END <<<
       ========================================================== */
}