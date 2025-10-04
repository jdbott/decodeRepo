package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class OneMotorShooter {
    public DcMotorEx motor;

    private static final double wheelCircumference = 0.0762; // meters
    private static final double shaftCircumference = 0.008; // meters
    private static final double TICKS_PER_REVOLUTION = 537.7;

    public void init(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(direction);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void spinMotor(double wheelVelocity) {
        double shaftRadius = shaftCircumference / (2 * Math.PI);
        double linearShaftVelocity = wheelVelocity * (shaftCircumference / wheelCircumference);
        double angularShaftVelocity = linearShaftVelocity / shaftRadius;
        motor.setVelocity(angularShaftVelocity, AngleUnit.RADIANS);
    }

    public double getCurrentVelocity() {
        if (motor == null) return 0;
        double angularVelocity = motor.getVelocity(AngleUnit.RADIANS);
        double shaftRadius = shaftCircumference / (2 * Math.PI);
        double linearShaftVelocity = angularVelocity * shaftRadius;
        return linearShaftVelocity * (wheelCircumference / shaftCircumference);
    }

    public void spinMotorRPM(double rpm) {
        motor.setVelocity(rpm * TICKS_PER_REVOLUTION / 60.0);
    }

    public double getCurrentVelocityRPM() {
        if (motor == null) return 0;
        return motor.getVelocity() * 60.0 / TICKS_PER_REVOLUTION;
    }

    public void stopMotor() {
        motor.setPower(0);
    }
}
