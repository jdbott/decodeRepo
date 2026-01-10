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

    // =========================
    // Turret auto-aim additions
    // =========================
    private final Turret turret = new Turret();

    // Same target point used in ScrimTele
    private static double TARGET_X = 0;
    private static double TARGET_Y = 142;

    // Turret is rotated 180° relative to ScrimTele setup
    private static final double TURRET_OFFSET_DEG = 180;

    // Shooting toggle (Gamepad 1)
    private boolean shootingActive = false;
    private boolean lastXButton = false; // edge memory

    // Intake toggle (Gamepad 2)
    private boolean intakeActive = false;
    private boolean lastSquareButton = false;

    // =========================
    // Manual turret override latch
    // =========================
    private boolean turretManualOverride = false;
    private long lastManualInputMs = 0;
    private static final long MANUAL_HOLD_MS = 4000;

    boolean lbPrev = false;

    private DcMotorEx shootMotor2; // mechanically linked

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
            // Robot pose
            // -----------------------------
            Pose pose = follower.getPose();
            double botX = pose.getX();
            double botY = pose.getY();
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());

            // -----------------------------
            // Drive (existing)
            // -----------------------------
            drive(robotHeadingDeg);

            // -----------------------------
            // Intake toggle (existing, gamepad2 square)
            // -----------------------------
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

            // =========================
            // Shooting toggle
            // =========================
            boolean shootToggleBtn = gamepad2.cross;
            if (shootToggleBtn && !lastXButton) {
                shootingActive = !shootingActive;
            }
            lastXButton = shootToggleBtn;

            // -----------------------------
            // Target angle calculations (used for telemetry + auto)
            // -----------------------------
            double dx = TARGET_X - botX;
            double dy = TARGET_Y - botY;
            double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
            double turretAngleNeededDeg = normalize180(angleToTargetDeg - robotHeadingDeg);
            double autoCmdDeg = normalize180(turretAngleNeededDeg + TURRET_OFFSET_DEG);

            // =========================
            // Manual turret adjust (pauses auto)
            // =========================
            boolean manualInput = false;

            if (gamepad2.dpad_left) {
                turret.setAngle(turret.getTargetAngle() - 1);
                manualInput = true;
            }
            if (gamepad2.dpad_right) {
                turret.setAngle(turret.getTargetAngle() + 1);
                manualInput = true;
            }

            if (manualInput) {
                turretManualOverride = true;
                lastManualInputMs = System.currentTimeMillis();
            }

            // Zero turret: clears override and resumes auto behavior
            if (gamepad2.share) {
                turret.zeroTurret();
                turretManualOverride = false;
                lastManualInputMs = 0;
                gamepad2.rumble(500);
            }

            // If we haven't manually touched it recently, allow auto again
            if (turretManualOverride && (System.currentTimeMillis() - lastManualInputMs) > MANUAL_HOLD_MS) {
                turretManualOverride = false;
            }

            // =========================
            // Turret auto-aim (gated)
            // =========================
            if (!turretManualOverride) {
                if (shootingActive) {
                    turret.setAngle(autoCmdDeg);
                } else {
                    turret.setAngle(0);
                }
            }

            turret.update();

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

            telemetry.addData("ShootMode", shootingActive);
            telemetry.addData("ManualOverride", turretManualOverride);
            telemetry.addData("Pose", pose.toString());
            telemetry.addData("AngleToTargetDeg", angleToTargetDeg);
            telemetry.addData("AutoCmdDeg", shootingActive ? autoCmdDeg : 0);
            telemetry.update();
        }
    }

    // -----------------------------
    // Extracted drive logic (existing)
    // -----------------------------
    private void drive(double robotHeadingDeg) {
        double rotatedX;
        double rotatedY;
        double x;
        double y;
        double rx;

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
}