package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@TeleOp(name = "V2 TELEOP BLUE")
public class V2TeleBlue extends LinearOpMode {

    // --- Subsystems / hardware ---
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlate basePlate;

    // Drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Shooter (mechanically linked)
    private DcMotorEx shootMotor2;

    // Turret object (kept as a field so we can init once)
    private final Turret turret = new Turret();

    // Edge memory for basePlate LB trigger
    private boolean lbPrev = false;

    @Override
    public void runOpMode() throws InterruptedException {

        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        // -----------------------------
        // Follower init
        // -----------------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(35, 72, Math.toRadians(180)));
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
        basePlate = new BasePlate(hardwareMap);

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
        turret.setLimits(-160, 200);
        turret.update();

        telemetry.addLine("Initialized. Press START to begin.");
        telemetry.update();

        while (opModeInInit()) {
            turret.update();
            basePlate.update();
        }

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            follower.update();

            // -----------------------------
            // Robot pose (for turret + telemetry)
            // -----------------------------
            Pose pose = follower.getPose();

            // -----------------------------
            // Drive (existing)
            // -----------------------------
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());
            drive(robotHeadingDeg);

            // -----------------------------
            // Intake toggle (existing, gamepad2 square)
            // -----------------------------
            // Kept in-line, unchanged (your request was only to extract turret logic)
            // If you want, we can extract intake similarly later.
            // NOTE: You were using "square" already.
            // -----------------------------
            // Intake toggle state is maintained inside the turret method? No.
            // We'll keep the intake toggle fields inside the turret method per your request? No.
            // You requested only turret logic extraction.
            // -----------------------------
            // So we keep intake toggles here (unchanged).
            // -----------------------------
            // Intake toggle variables (local static-ish behavior) moved to fields? No—unchanged in your original.
            // We'll implement exactly as your original had them, but as locals we need persistent state.
            // Therefore, we keep them as fields inside run loop? Not possible.
            // Best minimal change: promote to fields? That violates your request? It's not turret logic.
            // We'll keep them as fields at class scope? You didn't ask, so I will keep as fields at class scope.
            // (See fields below.)
            // -----------------------------
            handleIntakeToggle();

            // -----------------------------
            // Turret logic (extracted)
            // -----------------------------
            handleTurretTrackingAndControl(pose);

            // -----------------------------
            // BasePlate controls (existing)
            // -----------------------------
            boolean lbNow = gamepad2.left_bumper;
            if (lbNow && !lbPrev && !basePlate.isShootBusy()) {
                basePlate.startFullShoot();
            }
            lbPrev = lbNow;

            if (gamepad1.dpad_up) {
                basePlate.prepShootOnly();
            }
            if (gamepad1.dpad_down) {
                basePlate.startShootFromPrep();
            }
            if (gamepad1.dpad_left) {
                basePlate.startShootFromPush1Wait();
            }
            if (gamepad2.right_bumper) {
                basePlate.gateHoldBall1();
                shootMotor2.setPower(0.785);
            }

            basePlate.update();

            telemetry.update();
        }
    }

    // =========================================================
    // Intake toggle (unchanged behavior; promoted to a method
    // because it needs persistent edge memory variables).
    // =========================================================
    private boolean intakeActive = false;
    private boolean lastSquareButton = false;

    private void handleIntakeToggle() {
        boolean squareButton = gamepad2.square;
        if (squareButton && !lastSquareButton) {
            intakeActive = !intakeActive;
            if (intakeActive) {
                intake.intakeIn();
            } else {
                intake.intakeStop();
            }
        }
        lastSquareButton = squareButton;
    }

    // =========================================================
    // Turret control and tracking (ALL turret logic lives here)
    // =========================================================
    private void handleTurretTrackingAndControl(Pose pose) {
        // =========================
        // Configuration / constants
        // =========================
        final double TARGET_X = 0;
        final double TARGET_Y = 142;

        // Turret is rotated 180° relative to ScrimTele setup
        final double TURRET_OFFSET_DEG = 180;

        // Physical turret window
        final double TURRET_MIN_DEG = -160.0;
        final double TURRET_MAX_DEG =  200.0;

        // Manual override decay time
        final long MANUAL_HOLD_MS = 4000;

        // =========================
        // Persistent state (turret-specific)
        // =========================
        // Because you requested all turret logic not at the top of the class,
        // we keep the turret state as static-like fields inside this method
        // by storing them in member variables and only touching them here.
        // (Java cannot have true static locals.)
        //
        // These fields are declared at the bottom of the class.
        // =========================

        // -----------------------------
        // Pose / headings
        // -----------------------------
        double botX = pose.getX();
        double botY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // =========================
        // Shooting toggle (gamepad2 cross)
        // =========================
        boolean shootToggleBtn = gamepad2.cross;
        if (shootToggleBtn && !turretLastShootToggleBtn) {
            turretShootingActive = !turretShootingActive;
        }
        turretLastShootToggleBtn = shootToggleBtn;

        // -----------------------------
        // Compute raw desired turret command (deg)
        // -----------------------------
        double dx = TARGET_X - botX;
        double dy = TARGET_Y - botY;

        double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngleNeededDeg = normalize180(angleToTargetDeg - robotHeadingDeg);

        // Your existing logic: normalize180 and then apply offset
        double rawAutoCmdDeg = normalize180(turretAngleNeededDeg + TURRET_OFFSET_DEG);

        // -----------------------------
        // Helper: wrap a desired angle into the turret window via +/-360
        // preferring the candidate closest to a reference (current target).
        // -----------------------------
        double safeAutoCmdDeg = wrapIntoTurretWindow(rawAutoCmdDeg, turret.getTargetAngle(), TURRET_MIN_DEG, TURRET_MAX_DEG);

        // =========================
        // Manual turret adjust (pauses auto)
        // =========================
        boolean manualInput = false;

        if (gamepad2.dpad_left) {
            double desired = turret.getTargetAngle() - 1.0;
            turret.setAngle(wrapIntoTurretWindow(desired, turret.getTargetAngle(), TURRET_MIN_DEG, TURRET_MAX_DEG));
            manualInput = true;
        }

        if (gamepad2.dpad_right) {
            double desired = turret.getTargetAngle() + 1.0;
            turret.setAngle(wrapIntoTurretWindow(desired, turret.getTargetAngle(), TURRET_MIN_DEG, TURRET_MAX_DEG));
            manualInput = true;
        }

        if (manualInput) {
            turretManualOverride = true;
            turretLastManualInputMs = System.currentTimeMillis();
        }

        // Zero turret: clears override and resumes auto behavior
        if (gamepad2.share) {
            turret.zeroTurret();
            turretManualOverride = false;
            turretLastManualInputMs = 0;
            gamepad2.rumble(500);
        }

        // If we haven't manually touched it recently, allow auto again
        if (turretManualOverride && (System.currentTimeMillis() - turretLastManualInputMs) > MANUAL_HOLD_MS) {
            turretManualOverride = false;
        }

        // =========================
        // Turret auto-aim (gated)
        // =========================
        if (!turretManualOverride) {
            if (turretShootingActive) {
                turret.setAngle(safeAutoCmdDeg);
            } else {
                // Keep 0 legal via the same mapping (uniform behavior)
                turret.setAngle(wrapIntoTurretWindow(0.0, turret.getTargetAngle(), TURRET_MIN_DEG, TURRET_MAX_DEG));
            }
        }

        turret.update();

        // -----------------------------
        // Telemetry (turret-specific)
        // -----------------------------
        telemetry.addData("ShootMode", turretShootingActive);
        telemetry.addData("ManualOverride", turretManualOverride);
        telemetry.addData("Pose", pose.toString());
        telemetry.addData("AngleToTargetDeg", angleToTargetDeg);
        telemetry.addData("AutoCmdDeg", turretShootingActive ? safeAutoCmdDeg : 0);
        telemetry.addData("TurretTargetDeg", turret.getTargetAngle());
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
    // Extracted drive logic (existing)
    // -----------------------------
    private void drive(double robotHeadingDeg) {
        double trigger = Range.clip(1 - gamepad2.right_trigger, 0.2, 1);

        if (!(gamepad2.left_trigger > 0.5)) {
            follower.setTeleOpDrive(
                    gamepad2.left_stick_y * trigger,
                    gamepad2.left_stick_x * trigger,
                    -gamepad2.right_stick_x * trigger,
                    false);
        } else {
            follower.setTeleOpDrive(
                    -gamepad2.left_stick_y * trigger,
                    -gamepad2.left_stick_x * trigger,
                    -gamepad2.right_stick_x * trigger,
                    true);
        }

        if (gamepad2.circle) {
            follower.setPose(new Pose(24.23, 125.24, Math.toRadians(144)));
            gamepad2.rumble(500);
        }
    }

    private double normalize180(double a) {
        a = ((a + 180) % 360 + 360) % 360 - 180;
        return a;
    }

    // =========================================================
    // Turret state fields (kept out of the "top" logic)
    // =========================================================
    private boolean turretShootingActive = false;
    private boolean turretLastShootToggleBtn = false;

    private boolean turretManualOverride = false;
    private long turretLastManualInputMs = 0;
}