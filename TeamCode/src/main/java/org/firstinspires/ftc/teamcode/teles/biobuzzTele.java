package org.firstinspires.ftc.teamcode.teles;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric No Strafe", group = "TeleOp")
public class biobuzzTele extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware Map ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Reverse right side so positive power = forward for all motors
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // --- IMU ---
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);

        waitForStart();

        while (opModeIsActive()) {
            // --- Driver Inputs ---
            double forwardInput = -gamepad1.left_stick_y;   // Field forward/back
            double turnInput    = gamepad1.left_stick_x     // Strafe input -> TURN
                    + gamepad1.right_stick_x;   // Normal turn input

            // --- Field-Centric Rotation ---
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the forward command by the robot's heading.
            // Strafe (x) is forced to 0 — it is NOT an option on this controller.
            double robotForward = forwardInput * Math.cos(-heading);

            // --- Tank-Style Outputs (no strafe) ---
            double denominator = Math.max(Math.abs(robotForward) + Math.abs(turnInput), 1.0);

            double lf = (robotForward + turnInput) / denominator;
            double rf = (robotForward - turnInput) / denominator;
            double lb = (robotForward + turnInput) / denominator;
            double rb = (robotForward - turnInput) / denominator;

            leftFront.setPower(lf);
            rightFront.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);

            // --- Telemetry ---
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("Field Forward", "%.2f", robotForward);
            telemetry.addData("Turn", "%.2f", turnInput);
            telemetry.addLine("Strafe: DISABLED");
            telemetry.update();
        }
    }
}