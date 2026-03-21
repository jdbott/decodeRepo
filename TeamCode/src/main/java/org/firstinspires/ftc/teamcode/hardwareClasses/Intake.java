package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx motor;
    private final double INTAKE_POWER = 1;
    private final double OUTTAKE_POWER = -1;

    // Constructor
    public Intake(HardwareMap hardwareMap) {
        // Initialize motor
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Spin intake inward (pull artifact in)
    public void intakeIn() {
        motor.setPower(INTAKE_POWER);
    }

    public void intakeInSlow() {motor.setPower(INTAKE_POWER - 0.35);}

    // Spin intake outward (eject artifact)
    public void intakeOut() {
        motor.setPower(OUTTAKE_POWER);
    }

    // Stop intake motor
    public void intakeStop() {
        motor.setPower(0);
    }
}
