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

import org.firstinspires.ftc.teamcode.hardwareClasses.BasePlate;
import org.firstinspires.ftc.teamcode.hardwareClasses.Gantry;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.hardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@TeleOp(name = "V2 TELEOP RED")
public class V2TeleRed extends LinearOpMode {

    // --- Subsystems / hardware ---
    private Follower follower;
    private Intake intake;
    private Gantry gantry;
    private BasePlate basePlate;
    private Shooter shooter;

    // Drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Shooter (mechanically linked) (kept for reference; ShooterV2 owns the actual control)
    @SuppressWarnings("unused")
    private DcMotorEx shootMotor2;
    @SuppressWarnings("unused")
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

    @Override
    public void runOpMode() throws InterruptedException {

        // -----------------------------
        // Follower init
        // -----------------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-25, 85, Math.toRadians(180)));
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

        shooter = new Shooter();
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

            // -----------------------------
            // Intake toggle + FSM
            // -----------------------------

            if (gamepad2.right_bumper) {
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

            if (gamepad2.left_trigger > 0.5) {
                basePlate.startFullShoot();
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
        final double TARGET_X = 144;
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

            if (turretShootingActive) {
                // RPM now should be distance-based (from your earlier change)
                // We'll compute distance below and set RPM there.
            } else {
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

        if (gamepad2.dpad_left) {
            double desired = turret.getTargetAngle() - 1.0;
            turret.setAngle(wrapIntoTurretWindow(desired, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG));
            manualInput = true;
        }

        if (gamepad2.dpad_right) {
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
            // Flywheel RPM selection by distance (your 3-bucket logic)
            double rpmCmd = getShooterRPMForDistance(distanceToTarget);
            shooter.setTargetRPM(rpmCmd);
            if (!(gamepad2.right_trigger > 0.5)) {
                turret.setAngle(0);
            }
            if (!turretManualOverride && !(gamepad2.right_trigger > 0.5)) {
                turret.setAngle(safeCmdDeg);
            }
        } else {
            // Not tracking: stop shooter; park turret (unless manual override)
            // (Shooter stop handled by toggle block, but this keeps it safe if state drifts)
            // shooterV2.stop();

            if (!turretManualOverride) {
                double park = wrapIntoTurretWindow(135, turretCurrentDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
                turret.setAngle(park);
            }
        }

        turret.update();

        // =========================
        // Telemetry
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
        telemetry.addData("ShooterRPM", shooter.getMasterRPM());
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
                    -gamepad1.left_stick_y * trigger,
                    -gamepad1.left_stick_x * trigger,
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
            follower.setPose(new Pose(144-24.4, 126, Math.toRadians(180-142)));
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
        // Change if your BasePlate method name differs
        basePlate.gateUp();
    }

    private void gateOnePosition() {
        basePlate.gateHoldBall1();
    }

    private void gateDown() {
        // IMPORTANT: this is probably NOT correct in your current codebase.
        // Replace with your true "gate down/closed" method when you confirm it.
        // For now it matches what you had, but you should fix it.
        basePlate.gateHoldBall1();
    }
}
