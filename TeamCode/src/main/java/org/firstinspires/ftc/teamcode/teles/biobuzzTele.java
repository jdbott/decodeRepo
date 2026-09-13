package org.firstinspires.ftc.teamcode.teles;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "BioBuzz Tele", group = "TeleOp")
public class biobuzzTele extends LinearOpMode {

    // PS5 (DualSense) mapping: Cross = a, Circle = b, Square = x, Triangle = y,
    // R2 = right_trigger, L2 = left_trigger, D-pad = dpad_up/down/left/right.
    // Sticks: left stick drive/strafe, right stick X turn.

    // TODO: replace with real Control Hub config names once ports are assigned
    private static final String BOX_LIFT_NAME = "boxLift";
    private static final String CANISTER_SERVO_NAME = "canisterServo";
    private static final String GATE_SERVO_NAME = "gateServo";
    private static final String SHOOTER_NAME = "shooter";

    // Shooter: 4000 RPM target, fixed ~70 deg mechanical launch angle
    private static final double SHOOTER_TARGET_RPM = 4000.0;
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // TODO: set to shooter motor CPR
    private static final double LAUNCH_ANGLE_DEG = 70.0;

    // Box lift presets (ticks) — tune to mechanism
    private static final int BOX_DOWN_TICKS = 0;
    private static final int BOX_UP_TICKS = 1000; // TODO: tune
    private static final double BOX_POWER = 0.8;

    // Canister servo sweep, degrees mapped onto 0-1 range as deg/360
    private static final double CANISTER_A_DEG = 210.0;
    private static final double CANISTER_B_DEG = 330.0;

    // Gate servo positions (up = open/shoot, down = closed)
    private static final double GATE_CLOSED_POS = 0.0; // TODO: tune
    private static final double GATE_OPEN_POS = 1.0;   // TODO: tune

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotorEx boxLift, shooter;
    private Servo canisterServo, gateServo;
    private IMU imu;

    private static double degToPos(double deg) {
        return deg / 360.0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        boxLift = hardwareMap.get(DcMotorEx.class, BOX_LIFT_NAME);
        boxLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boxLift.setTargetPosition(BOX_DOWN_TICKS);
        boxLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boxLift.setPower(BOX_POWER);

        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        canisterServo = hardwareMap.get(Servo.class, CANISTER_SERVO_NAME);
        gateServo = hardwareMap.get(Servo.class, GATE_SERVO_NAME);

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);

        boolean boxUp = false;
        boolean canisterAtB = false;
        boolean prevDpadUp = false;
        boolean prevDpadRight = false;
        double shooterVel = SHOOTER_TARGET_RPM / 60.0 * SHOOTER_TICKS_PER_REV;

        canisterServo.setPosition(degToPos(CANISTER_A_DEG));
        gateServo.setPosition(GATE_CLOSED_POS);

        waitForStart();

        while (opModeIsActive()) {
            // Shooter spins the whole teleop
            shooter.setVelocity(shooterVel);

            // D-pad up toggles box lift up/down
            if (gamepad1.dpad_up && !prevDpadUp) {
                boxUp = !boxUp;
                boxLift.setTargetPosition(boxUp ? BOX_UP_TICKS : BOX_DOWN_TICKS);
            }
            prevDpadUp = gamepad1.dpad_up;

            // D-pad right toggles canister 210 deg <-> 330 deg
            if (gamepad1.dpad_right && !prevDpadRight) {
                canisterAtB = !canisterAtB;
                canisterServo.setPosition(degToPos(canisterAtB ? CANISTER_B_DEG : CANISTER_A_DEG));
            }
            prevDpadRight = gamepad1.dpad_right;

            // Shoot held (R2 or Cross) -> gate up (open) so balls feed the shooter
            boolean shooting = gamepad1.right_trigger > 0.3 || gamepad1.a;
            gateServo.setPosition(shooting ? GATE_OPEN_POS : GATE_CLOSED_POS);

            // Field-centric mecanum with strafe
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
            leftFront.setPower((rotY + rotX + turn) / denominator);
            leftBack.setPower((rotY - rotX + turn) / denominator);
            rightFront.setPower((rotY - rotX - turn) / denominator);
            rightBack.setPower((rotY + rotX - turn) / denominator);

            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("Box", boxUp ? "UP" : "DOWN");
            telemetry.addData("Canister (deg)", canisterAtB ? CANISTER_B_DEG : CANISTER_A_DEG);
            telemetry.addData("Gate", shooting ? "OPEN" : "CLOSED");
            telemetry.addData("Shooter target (RPM)", "%.0f", SHOOTER_TARGET_RPM);
            telemetry.addData("Shooter vel (t/s)", "%.0f", shooter.getVelocity());
            telemetry.addData("Launch angle (deg)", "%.0f", LAUNCH_ANGLE_DEG);
            telemetry.update();
        }
    }
}
