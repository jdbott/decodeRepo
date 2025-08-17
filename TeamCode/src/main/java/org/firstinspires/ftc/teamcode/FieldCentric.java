package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name = "FieldCentric")
public class FieldCentric extends OpMode {

    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor frontLeft;

    BHI260IMU imu;



    @Override
    public void init() {
        //modify this entire section as needed
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize();
        imu.resetYaw();

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addData("Current IMU Angle", botHeading);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        //these are the x and y vectors from the robot's point of view
        //the formulas are derived using sine and cosine angle addition formulas.
        //bot heading is negative as imu considers counterclockwise positive.
        double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //now we just the regular mecanum formula.
        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1.0);
        double frontLeftPower = (rotatedY + rotatedX + rx) / denominator;
        double backLeftPower = (rotatedY - rotatedX + rx) / denominator;
        double frontRightPower = (rotatedY - rotatedX - rx) / denominator;
        double backRightPower = (rotatedY + rotatedX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}
