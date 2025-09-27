package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx intakeMotor;
    private final ColorV3 colorSensor;

    // Default power values (tunable)
    private double intakePower = 0.8;
    private double outtakePower = -0.8;

    // Constructor
    public Intake(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        // Initialize motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, motorName);
        intakeMotor.setDirection(direction);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize your wrapped Rev Color Sensor V3
        colorSensor = new ColorV3(hardwareMap);
    }

    // Spin intake inward (pull artifact in)
    public void intakeIn() {
        intakeMotor.setPower(intakePower);
    }

    // Spin intake outward (eject artifact)
    public void intakeOut() {
        intakeMotor.setPower(outtakePower);
    }

    // Stop intake motor
    public void stop() {
        intakeMotor.setPower(0);
    }

    // Optional: dynamically adjust intake power
    public void setIntakePower(double power) {
        intakePower = Math.abs(power);
        outtakePower = -Math.abs(power);
    }

    // === COLOR SENSOR METHODS ===

    // Get both proximity and artifact color (uses your ColorV3 class logic)
    public String artifactStatus() {
        return colorSensor.proximityAndColor();
    }

    // Get only proximity in inches
    public double artifactProximity() {
        return colorSensor.proximity();
    }

    // Get just detected artifact color
    public String artifactColor() {
        return colorSensor.sampleColor();
    }

    // Check if color sensor is connected
    public boolean isColorSensorConnected() {
        return colorSensor.isConnected();
    }
}
