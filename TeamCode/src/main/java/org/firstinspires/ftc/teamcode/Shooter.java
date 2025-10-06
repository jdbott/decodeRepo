package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

public class Shooter {
    public List<DcMotorEx> motors = new ArrayList<>();
    public Servo hoodServo;

    private static final double wheelCircumference = 0.0762; // meters
    private static final double shaftCircumference = 0.008; // meters
    private static final double C1 = 232; // mm
    private static final double C2 = 32; // mm
    private static final double C3 = 200; // mm
    private static final double C4 = 51.24; // mm
    private static final double minAngle = 30; // degrees
    private static final double maxAngle = 180; // degrees

    private static final double TICKS_PER_REVOLUTION = 537.7;

    public void init(HardwareMap hardwareMap, String[] motorNames, DcMotorSimple.Direction[] directions, DcMotorEx.RunMode[] runModes, String hoodServoName) {
        motors.clear();
        for (int i = 0; i < motorNames.length; i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorNames[i]);
            motor.setDirection(directions[i]);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(runModes[i]);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motors.add(motor);
        }
        hoodServo = hardwareMap.get(Servo.class, hoodServoName);
        hoodServo.setDirection(Servo.Direction.FORWARD);
        hoodServo.setPosition(0);
    }

    public void spinMotors(double wheelVelocity) {
        double shaftRadius = shaftCircumference / (2 * Math.PI);
        double linearShaftVelocity = wheelVelocity * (shaftCircumference / wheelCircumference);
        double angularShaftVelocity = linearShaftVelocity / shaftRadius;
        for (DcMotorEx motor : motors) {
            motor.setVelocity(angularShaftVelocity, AngleUnit.RADIANS);
        }
    }

    public double getCurrentVelocity() {
        if (motors.isEmpty()) return 0;
        double angularVelocity = motors.get(0).getVelocity(AngleUnit.RADIANS);
        double shaftRadius = shaftCircumference / (2 * Math.PI);
        double linearShaftVelocity = angularVelocity * shaftRadius;
        return linearShaftVelocity * (wheelCircumference / shaftCircumference);
    }

    public void spinMotorsRPM(double rpm) {
        // ticks per second
        for (DcMotorEx motor : motors) {
            motor.setVelocity(rpm * TICKS_PER_REVOLUTION / 60.0);
        }
    }

    public double getCurrentVelocityRPM() {
        if (motors.isEmpty()) return 0;
        return motors.get(0).getVelocity() * 60.0 / TICKS_PER_REVOLUTION;
    }

    public void stopMotors() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    public void setHoodAngle(double deltaAngle) {
        double x2 = Math.pow(C3, 2) + Math.pow(C4, 2) - 2 * C3 * C4 * Math.cos(Math.toRadians(deltaAngle));
        double gamma = Math.toDegrees(Math.acos((x2 - Math.pow(C1, 2) - Math.pow(C2, 2)) / (-2 * C1 * C2)));
        gamma = Range.clip(gamma, minAngle, maxAngle);
        hoodServo.setPosition(gamma / 180.0); // Normalize to [0, 1]
    }
}
