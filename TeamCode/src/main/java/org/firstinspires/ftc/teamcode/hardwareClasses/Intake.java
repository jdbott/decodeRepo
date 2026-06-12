package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Intake {

    private final DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void in() {
        intakeMotor.setPower(1.0);
    }

    public void out() {
        intakeMotor.setPower(-1.0);
    }

    public void stop() {
        intakeMotor.setPower(0.0);
    }
}
