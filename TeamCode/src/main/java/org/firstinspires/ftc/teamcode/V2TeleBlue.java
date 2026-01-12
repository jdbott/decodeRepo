package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotorEx shootMotor1;

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

    // Shooter power (set this to whatever you actually want)
    private static final double SHOOTER_POWER = .94;

    @Override
    public void runOpMode() throws InterruptedException {

        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shootMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shootMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

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

            Pose pose = follower.getPose();

            // -----------------------------
            // Drive (existing)
            // -----------------------------
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());
            drive(robotHeadingDeg);

            // -----------------------------
            // Intake toggle + FSM (NEW)
            // -----------------------------
            handleIntakeToggleAndFSM();

            // -----------------------------
            // Gate toggle on RB (NEW)
            // (Intake FSM will override this while intake is ON or shutting down)
            // -----------------------------
            handleGateToggle();

            // -----------------------------
            // Turret logic (extracted)
            // (Cross toggle now also spins flywheels)
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

            basePlate.update();
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.addData("IntakeState", intakeState);
            telemetry.addData("GateMode", gateToggleAllTheWayUp ? "ALL_UP" : "ONE_POS");
            telemetry.addData("ShootMode", turretShootingActive);
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
            } else {
                // Begin shutdown sequence
                // Gate goes down now; intake stops after delay
                intakeState = IntakeState.SHUTDOWN_WAIT;
                intakeShutdownStartMs = System.currentTimeMillis();
            }
        }
        lastSquareButton = squareButton;

        // FSM execution
        switch (intakeState) {
            case OFF:
                // Ensure intake is off (gate is handled by RB toggle or whatever default you prefer)
                intake.intakeStop();
                break;

            case ON:
                // Force gate ALL THE WAY UP while intake is running
                intake.intakeIn();
                gateAllTheWayUp();
                break;

            case SHUTDOWN_WAIT:
                // Gate DOWN immediately, then wait before stopping intake motor
                gateDown();

                if (System.currentTimeMillis() - intakeShutdownStartMs >= INTAKE_SHUTDOWN_DELAY_MS) {
                    intake.intakeStop();
                    intakeState = IntakeState.OFF;
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
        // If intake is ON or shutting down, gate is controlled by the intake FSM.
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

    // =========================================================
    // Turret control and tracking (ALL turret logic lives here)
    // - Cross toggles turret tracking AND flywheels
    // =========================================================
    private void handleTurretTrackingAndControl(Pose pose) {
        // =========================
        // Configuration / constants
        // =========================
        final double TARGET_X = 9;
        final double TARGET_Y = 142;

        // Turret is rotated 180° relative to ScrimTele setup
        final double TURRET_OFFSET_DEG = 180;

        // Physical turret window
        final double TURRET_MIN_DEG = -160.0;
        final double TURRET_MAX_DEG =  200.0;

        // Manual override decay time
        final long MANUAL_HOLD_MS = 4000;

        // -----------------------------
        // Pose / headings
        // -----------------------------
        double botX = pose.getX();
        double botY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // =========================
        // Shooting toggle (gamepad2 cross)
        // ALSO controls flywheel motors now
        // =========================
        boolean shootToggleBtn = gamepad2.cross;
        if (shootToggleBtn && !turretLastShootToggleBtn) {
            turretShootingActive = !turretShootingActive;

            if (turretShootingActive) {
                // Turn ON flywheels when enabling tracking
                shootMotor2.setPower(SHOOTER_POWER);
                shootMotor1.setPower(SHOOTER_POWER);
            } else {
                // Turn OFF flywheels when disabling tracking
                shootMotor2.setPower(0.0);
                shootMotor1.setPower(0.0);
            }
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

        // Wrap into turret window, favoring candidate closest to current target
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
                // When not tracking, park wherever you want (you had 135 here)
                turret.setAngle(wrapIntoTurretWindow(135, turret.getTargetAngle(), TURRET_MIN_DEG, TURRET_MAX_DEG));
            }
        }

        turret.update();

        // -----------------------------
        // Telemetry (turret-specific)
        // -----------------------------
        telemetry.addData("ManualOverride", turretManualOverride);
        telemetry.addData("AngleToTargetDeg", angleToTargetDeg);
        telemetry.addData("AutoCmdDeg", turretShootingActive ? safeAutoCmdDeg : 0);
        telemetry.addData("TurretTargetDeg", turret.getTargetAngle());
        telemetry.addData("Flywheels", turretShootingActive ? "ON" : "OFF");
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
            follower.setPose(new Pose(27.9, 125.25, Math.toRadians(141.75)));
            gamepad2.rumble(500);
        }
    }

    private double normalize180(double a) {
        a = ((a + 180) % 360 + 360) % 360 - 180;
        return a;
    }

    // =========================================================
    // Gate helpers
    // IMPORTANT: adjust these 3 methods to match your BasePlate API.
    // I used reasonable names based on what you've shown so far.
    // =========================================================
    private void gateAllTheWayUp() {
        // If your BasePlate method name differs, change it here.
        // Example possibilities: basePlate.gateUp(); basePlate.gateAllUp(); basePlate.gateOpen();
        basePlate.gateUp(); // <-- likely exists; if not, rename to your actual method
    }

    private void gateOnePosition() {
        // You already use this method in your codebase
        basePlate.gateHoldBall1();
    }

    private void gateDown() {
        // If your BasePlate method name differs, change it here.
        // Example possibilities: basePlate.gateDown(); basePlate.gateClosed();
        basePlate.gateHoldBall1(); // <-- likely exists; if not, rename to your actual method
    }

    // =========================================================
    // Turret state fields
    // =========================================================
    private boolean turretShootingActive = false;
    private boolean turretLastShootToggleBtn = false;

    private boolean turretManualOverride = false;
    private long turretLastManualInputMs = 0;
}