package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwareClasses.Box2Extension;

@Config
@TeleOp(name = "BioBuzz Tele", group = "TeleOp")
public class biobuzzTele extends LinearOpMode {

    // PS5 (DualSense) mapping: Cross = a, Circle = b, Square = x, Triangle = y,
    // R2 = right_trigger, L2 = left_trigger, D-pad = dpad_up/down/left/right.
    // Sticks: left stick drive/strafe, right stick X turn.

    private static final String INTAKE_MOTOR_NAME = "intake";
    private static final String SHOOTER_NAME = "shooter";
    private static final String GATE_SERVO_NAME = "gate";
    private static final String BOX_MOTOR_NAME = "box";
    private static final String ARM_SERVO_NAME = "arm";

    // Shooter velocity PID + feedforward, live-tunable from FTC Dashboard (rad/s)
    private static final double SHOOTER_TICKS_PER_REV = 28.0;
    private static final double SHOOTER_RAD_PER_TICK = 2.0 * Math.PI / SHOOTER_TICKS_PER_REV;
    private static final double SHOOTER_VELOCITY_INCREMENT_RAD_PER_SEC = 5.0;
    private static final double SHOOTER_START_VELOCITY_RAD_PER_SEC = 265; // ~4000 RPM

    public static double kP = 0.0015;
    public static double kV = 0.0016;
    public static double kS = 0.05;

    // Gate servo positions (open = feed shooter, closed = block)
    private static final double GATE_CLOSED_POS = 0.55;
    private static final double GATE_OPEN_POS = 0.4;

    // Box tube (Box2Extension) travel, inches
    private static final double BOX_DOWN_INCHES = 0.0;
    private static final double BOX_UP_INCHES = -3.0;
    private static final double BOX_MAX_POWER = 0.4;

    // Arm servo positions
    private static final double ARM_STOWED_POS = 0.51;
    private static final double ARM_DEPLOYED_POS = 0;

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor intake;
    private DcMotorEx shooter;
    private Servo gate, arm;
    private Box2Extension boxTube;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        gate = hardwareMap.get(Servo.class, GATE_SERVO_NAME);
        arm = hardwareMap.get(Servo.class, ARM_SERVO_NAME);

        boxTube = new Box2Extension(hardwareMap, BOX_MOTOR_NAME, true);
        boxTube.setLimits(-3, 3);
        boxTube.setMaxPower(BOX_MAX_POWER);

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);

        boolean gateOpen = false;
        boolean lastRightBumper = false;
        boolean intakeRunning = true;
        boolean lastLeftBumper = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean boxExtended = false;
        boolean lastB = false;
        boolean armDeployed = false;
        boolean lastX = false;

        gate.setPosition(GATE_CLOSED_POS);
        arm.setPosition(ARM_STOWED_POS);

        double shooterTargetVelocity = SHOOTER_START_VELOCITY_RAD_PER_SEC;

        waitForStart();

        intake.setPower(1.0);

        while (opModeIsActive()) {
            // Right bumper toggles the gate open/closed
            boolean rightBumper = gamepad1.right_bumper;
            if (rightBumper && !lastRightBumper) {
                gateOpen = !gateOpen;
                gate.setPosition(gateOpen ? GATE_OPEN_POS : GATE_CLOSED_POS);
            }
            lastRightBumper = rightBumper;

            // Left bumper toggles the intake on/off
            boolean leftBumper = gamepad1.left_bumper;
            if (leftBumper && !lastLeftBumper) {
                intakeRunning = !intakeRunning;
                intake.setPower(intakeRunning ? 1.0 : 0.0);
            }
            lastLeftBumper = leftBumper;

            // D-pad up/down adjusts shooter target velocity in rad/s
            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp) {
                shooterTargetVelocity += SHOOTER_VELOCITY_INCREMENT_RAD_PER_SEC;
            }
            lastDpadUp = dpadUp;

            boolean dpadDown = gamepad1.dpad_down;
            if (dpadDown && !lastDpadDown) {
                shooterTargetVelocity = Math.max(0.0, shooterTargetVelocity - SHOOTER_VELOCITY_INCREMENT_RAD_PER_SEC);
            }
            lastDpadDown = dpadDown;

            double shooterVelocity = Math.abs(shooter.getVelocity()) * SHOOTER_RAD_PER_TICK;
            double shooterError = shooterTargetVelocity - shooterVelocity;
            double shooterFeedforward = kV * shooterTargetVelocity + kS;
            double shooterPower = Range.clip(shooterFeedforward + kP * shooterError, 0.0, 1.0);
            shooter.setPower(shooterPower);

            // B toggles the box tube between 0 and 3 inches
            boolean bButton = gamepad1.b;
            if (bButton && !lastB) {
                boxExtended = !boxExtended;
                boxTube.setTargetInches(boxExtended ? 2.5 : 0.0);
            }
            lastB = bButton;
            boxTube.update();

            // X toggles the arm between 0 and 0.4
            boolean xButton = gamepad1.x;
            if (xButton && !lastX) {
                armDeployed = !armDeployed;
                arm.setPosition(armDeployed ? ARM_DEPLOYED_POS : ARM_STOWED_POS);
            }
            lastX = xButton;

            if (gamepad1.dpad_left) {
                arm.setPosition(1);
            }

            // Field-centric mecanum with strafe
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double heading = 0;

            double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
            leftFront.setPower((rotY + rotX + turn) / denominator);
            leftBack.setPower((rotY - rotX + turn) / denominator);
            rightFront.setPower((rotY - rotX - turn) / denominator);
            rightBack.setPower((rotY + rotX - turn) / denominator);

            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("Intake", intakeRunning ? "ON" : "OFF");
            telemetry.addData("Gate", gateOpen ? "OPEN" : "CLOSED");
            telemetry.addData("Shooter Target Velocity (rad/s)", "%.1f", shooterTargetVelocity);
            telemetry.addData("Shooter Velocity (rad/s)", "%.1f", shooterVelocity);
            telemetry.addData("Box", boxExtended ? "UP (3 in)" : "DOWN (0 in)");
            telemetry.addData("Box Current (in)", "%.2f", boxTube.getCurrentPositionInches());
            telemetry.addData("Arm", armDeployed ? "DEPLOYED" : "STOWED");
            telemetry.update();
        }

        intake.setPower(0);
        shooter.setPower(0);
        boxTube.stop();
    }
}
