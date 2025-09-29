package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/*
Hi Addy. Before our next meeting, please create a hardware class for the shooter. You will need to
initialize the two motors and hood servo, then add code to set the two motors to spin in sync at a
target velocity, not RPM. You will also need to look at the cad to create a model for how the
linkage changes the hood angle non linearly, then have a method to set the angle of the hood.
Time permitting, you should also work on adding the turret code to the same hardware class,
factoring in the reduction (24:108) and using the encoder to spin the turret to any angle with a PID,
also clamping the min and max angle to avoid wire tangling.
 */

public class Shooter {
    private final List<DcMotorEx> motors = new ArrayList<>();

    private final Servo hoodServo;
    private static final double wheelCircumference = 0.0762; // meters

    private static final double C1 = 232; //mm
    private static final double C2 = 32; //mm
    private static final double C3 =200; //mm
    private static final double C4 = 51.24; //mm

    private static final double minAngle = 30; //degrees

    private static final double maxAngle = 180; //degrees


    public Shooter(HardwareMap hardwareMap, String[] motorNames, DcMotorSimple.Direction[] directions, String hoodServoName) {

        for (int i = 0; i < motorNames.length; i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorNames[i]);
            motor.setDirection(directions[i]);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motor);
        }
        hoodServo = hardwareMap.get(Servo.class, hoodServoName);
        hoodServo.setDirection(Servo.Direction.FORWARD);
        hoodServo.setPosition(0);
    }

    // Spin the output shaft of the wheel shooter at a target velocity in m/s
    public void spinMotors(double wheelVelocity) {
        double shaftCircumference = 0.008;
        double shaftRadius = shaftCircumference / (2 * Math.PI);
        for (DcMotorEx motor : motors) {
            // convert velocity from m/s at the wheel to m/s at the motor shaft
            double linearShaftVelocity = wheelVelocity * (shaftCircumference / wheelCircumference);
            // convert velocity from linear velocity in m/s to angular velocity in rad/s
            double angularShaftVelocity = linearShaftVelocity / shaftRadius;
            // set the motor velocity to power the motor shaft at the calculated angular velocity
            motor.setVelocity(angularShaftVelocity, AngleUnit.RADIANS);
            telemetry.addData("Motor Velocity (rad/s)", motor.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Target Velocity (m/s)", wheelVelocity);
            telemetry.update();
        }
    }

    public void setAngle(double deltaAngle) {
        double x2 = Math.pow(C3, 2) + Math.pow(C4, 2) - 2*C3*C4*Math.cos(Math.toRadians(deltaAngle));

        double gamma = Math.toDegrees(Math.acos((x2 - Math.pow(C1, 2) - Math.pow(C2, 2))/(-2*C1*C2)));

        gamma = Range.clip(gamma, minAngle, maxAngle);

        hoodServo.setPosition(gamma/180); // Normalize to [0, 1]

        telemetry.addData("Hood Angle (degrees)", gamma);
        telemetry.update();
    }


    public void stopMotors() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    public void setHoodAngle(Servo hoodServo, double angle) {


    }
}
