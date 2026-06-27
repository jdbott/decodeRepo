package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class offSeasonIntake {

    private static final String LEFT_MOTOR_NAME = "cool";
    private static final String RIGHT_MOTOR_NAME = "cool";

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    public offSeasonIntake(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME);
        rightMotor = hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void in() {
        setPower(1.0);
    }

    public void out() {
        setPower(-1.0);
    }

    public void stop() {
        setPower(0.0);
    }
}