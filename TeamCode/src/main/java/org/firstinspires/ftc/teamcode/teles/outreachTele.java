package org.firstinspires.ftc.teamcode.teles;

import org.firstinspires.ftc.teamcode.RobotConfig;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardwareClasses.Feeder;
import org.firstinspires.ftc.teamcode.hardwareClasses.Flywheel;
import org.firstinspires.ftc.teamcode.hardwareClasses.Hood;
import org.firstinspires.ftc.teamcode.hardwareClasses.Intake;

@TeleOp(name = "Outreach Tele")
public class outreachTele extends LinearOpMode {

    private Hood hood;
    private Feeder feeder;

    private Intake intake;
    private Flywheel flywheel;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    private boolean oneDriver = false;

    // Shot mode (manual toggle only; no odometry for auto FAR detection)
    private enum ShotMode {
        CLOSE,
        FAR
    }

    private ShotMode shotMode = ShotMode.CLOSE;

    // Fixed override shot settings
    private static final double FIXED_POWER_SHOT_VELOCITY_RAD = 440;
    private static final double FIXED_POWER_SHOT_HOOD_DEG = 53.5;

    // Intake toggle
    private boolean lastX = false;
    private boolean intakeToggleOn = false;

    // Feed sequence trigger
    private boolean lastB = false;

    private boolean lastRightBumper1 = false;
    private boolean lastRightBumper2 = false;

    // Timed shoot/feed sequence
    private final ElapsedTime sequenceTimer = new ElapsedTime();

    private enum FeedState {
        IDLE,
        WAIT_BEFORE_INTAKE,
        RUN_INTAKE,
        DONE
    }

    private FeedState feedState = FeedState.IDLE;

    // Hood angle state
    private double hoodAngleDeg = 50.0;

    // Flywheel target velocity state
    private double targetVelocityRad = 275;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);

        hood = new Hood(hardwareMap);
        feeder = new Feeder(hardwareMap);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new Flywheel(hardwareMap, battery);

        leftFront = hardwareMap.get(DcMotorEx.class, RobotConfig.DRIVE_LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, RobotConfig.DRIVE_LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConfig.DRIVE_RIGHT_FRONT);
        rightBack = hardwareMap.get(DcMotorEx.class, RobotConfig.DRIVE_RIGHT_BACK);

        // Match the directions Pedro's MecanumConstants uses
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        feeder.clutchOut();
        feeder.armBlock();
        intake.setPower(0.0);

        telemetry.addLine("Initialized");
        telemetry.addLine("Clutch out, arm blocked, intake off");
        telemetry.addData("Shot Mode", shotMode);
        telemetry.update();

        while (opModeInInit()) {
            if (gamepad1.x) {
                oneDriver = true;
            } else if (gamepad1.b) {
                oneDriver = false;
            }

            telemetry.addData("One Driver", oneDriver);
            telemetry.addData("Shot Mode", shotMode);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        //intake.setPower(1);
        //flywheel.setTargetVelocity(targetVelocityRad);

        while (opModeIsActive()) {
            drive();

            handleShotModeToggle();

            if (shotMode == ShotMode.FAR) {
                targetVelocityRad = FIXED_POWER_SHOT_VELOCITY_RAD;
                hoodAngleDeg = FIXED_POWER_SHOT_HOOD_DEG;
                hoodAngleDeg = hood.setAngle(hoodAngleDeg);
            } else {
                targetVelocityRad = 275;
                hoodAngleDeg = 50.0;
                hoodAngleDeg = hood.setAngle(hoodAngleDeg);
            }

            handleXToggle();
            handleBSequence();

            //updateFeedSequence();

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            telemetry.addData("Shot Mode", shotMode);
            telemetry.addData("Hood Angle (deg)", hoodAngleDeg);
            telemetry.addData("Flywheel Target (rad/s)", targetVelocityRad);
            telemetry.update();
        }

        flywheel.stop();
        intake.setPower(0.0);
    }

    private void drive() {
        // Left trigger = slow mode
        double trigger = Range.clip(1 - gamepad1.left_trigger, 0.2, 1);

        // Drivetrain sits 180° from the motor names: invert both translation axes, leave turn.
        double forward = gamepad1.left_stick_y * trigger;
        double strafe = -gamepad1.left_stick_x * trigger;
        double turn = -gamepad1.right_stick_x * trigger;

        double lf = forward + strafe + turn;
        double lb = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rb = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(lb),
                Math.max(Math.abs(rf), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);
    }

    private void handleShotModeToggle() {
        boolean rb1Pressed = gamepad1.left_bumper;
        boolean rb2Pressed = gamepad2.left_bumper;

        if (oneDriver) {
            if (rb1Pressed && !lastRightBumper1) {
                cycleShotMode();
                gamepad1.rumble(400);
            }
        } else {
            if (rb2Pressed && !lastRightBumper2) {
                cycleShotMode();
                gamepad2.rumble(400);
            }
        }

        lastRightBumper1 = rb1Pressed;
        lastRightBumper2 = rb2Pressed;
    }

    private void cycleShotMode() {
        if (shotMode == ShotMode.CLOSE) {
            shotMode = ShotMode.FAR;
        } else {
            shotMode = ShotMode.CLOSE;
        }
    }

    private void handleXToggle() {
        boolean xPressed;
        if (oneDriver) {
            xPressed = gamepad1.x;
        } else {
            xPressed = gamepad2.x;
        }

        /*
        if (xPressed && !lastX) {
            intakeToggleOn = !intakeToggleOn;

            if (feedState == FeedState.IDLE || feedState == FeedState.DONE) {
                if (intakeToggleOn) {
                    feeder.clutchOut();
                    feeder.armBlock();
                    intake.setPower(1.0);
                } else {
                    intake.setPower(0.0);
                }
            }
        }
         */

        lastX = xPressed;
    }

    private void handleBSequence() {
        boolean bPressed;
        if (oneDriver) {
            bPressed = gamepad1.right_bumper;
        } else {
            bPressed = gamepad2.right_bumper;
        }

        if (bPressed && !lastB) {
            intakeToggleOn = false;
            intake.setPower(0.0);

            feeder.armShoot();
            feeder.clutchIn();

            sequenceTimer.reset();
            feedState = FeedState.WAIT_BEFORE_INTAKE;
        }

        lastB = bPressed;
    }

    private void updateFeedSequence() {
        switch (feedState) {
            case IDLE:
            case DONE:
                break;

            case WAIT_BEFORE_INTAKE:
                if (sequenceTimer.seconds() >= 0.1) {
                    intake.setPower(1.0);
                    sequenceTimer.reset();
                    feedState = FeedState.RUN_INTAKE;
                }
                break;

            case RUN_INTAKE:
                if (sequenceTimer.seconds() >= 0.9) {
                    intake.setPower(1);
                    feeder.clutchOut();
                    feeder.armBlock();
                    feedState = FeedState.DONE;
                }
                break;
        }
    }
}