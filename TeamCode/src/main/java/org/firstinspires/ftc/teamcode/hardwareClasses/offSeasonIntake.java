package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class offSeasonIntake {

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    public offSeasonIntake(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_LEFT_MOTOR);
        rightMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_RIGHT_MOTOR);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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