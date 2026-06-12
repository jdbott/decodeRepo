package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void in() {
        motor.setPower(1.0);
    }

    public void out() {
        motor.setPower(-1.0);
    }

    public void stop() {
        motor.setPower(0.0);
    }
}
