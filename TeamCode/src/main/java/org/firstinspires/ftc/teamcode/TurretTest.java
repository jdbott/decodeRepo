package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Config
@Disabled
@TeleOp(name = "Turret Test")
public class TurretTest extends LinearOpMode {
    private Turret turret = new Turret();
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private double targetHeading = 0; // absolute field heading we want turret to maintain
    private double turretOffset = 0;  // compensation when robot spins past ±180

    public static double ff = 0.003;
    public static double kP = 0.008;

    @Override
    public void runOpMode() {
        // Initialize turret
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.setKP(kP);
        turret.setKF(ff);
        turret.setLimits(-180, 180);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));

        // Zero reference
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        targetHeading = orientation.getYaw(AngleUnit.DEGREES);

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

        telemetry.addLine("Turret + IMU Initialized");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        while (opModeIsActive()) {
            // D-pad sets new turret reference heading
            if (gamepad1.dpad_up) targetHeading = 0;
            if (gamepad1.dpad_left) targetHeading = -90;
            if (gamepad1.dpad_right) targetHeading = 90;
            if (gamepad1.dpad_down) targetHeading = 180;

            // Get robot yaw
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate difference between desired world heading and robot rotation
            double turretTarget = targetHeading + robotYaw + turretOffset;

            // Normalize to [-180,180]
            if (turretTarget > 180) turretTarget -= 360;
            if (turretTarget < -180) turretTarget += 360;

            // Compensate if robot crosses ±180
            if (turretTarget > 180 || turretTarget < -180) {
                turretOffset = (turretOffset > 0) ? turretOffset - 360 : turretOffset + 360;
                turretTarget = targetHeading - robotYaw + turretOffset;
            }

            double yawVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
            turret.setFeedforward(yawVel);

            // Apply to turret
            turret.setAngle(turretTarget);
            turret.update();

            turret.setKF(ff);
            turret.setKP(kP);

            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x * 1.1; // strafe correction
            double rx = gamepad1.right_stick_x; // rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            telemetry.addData("Target Field Heading", targetHeading);
            telemetry.addData("Robot Yaw", robotYaw);
            telemetry.addData("Turret Target Angle", turretTarget);
            telemetry.addData("Turret Actual", turret.getCurrentAngle());
            telemetry.update();
        }
    }
}