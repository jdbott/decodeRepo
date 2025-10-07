package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    private DcMotorEx masterMotor;
    private DcMotorEx followerMotor;

    private static final double TICKS_PER_REV = 28.0;

    private double kP = 0.00015;   // proportional gain
    private double kF = 0.00003;   // feedforward (power per tick/sec)

    private double targetRPM = 0.0;
    private double targetTPS = 0.0;

    public void init(HardwareMap hw,
                     String masterName, String followerName,
                     DcMotorSimple.Direction masterDir, DcMotorSimple.Direction followerDir) {

        masterMotor = hw.get(DcMotorEx.class, masterName);
        followerMotor = hw.get(DcMotorEx.class, followerName);

        masterMotor.setDirection(masterDir);
        followerMotor.setDirection(followerDir);

        masterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        masterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        followerMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        masterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        followerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /** Set desired wheel speed in RPM */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        targetTPS = rpm * TICKS_PER_REV / 60.0;
    }

    /** Read master encoder velocity (ticks/sec) */
    public double getMasterVelocityTPS() {
        return masterMotor.getVelocity();
    }

    /** Read master RPM */
    public double getMasterRPM() {
        return getMasterVelocityTPS() * 60.0 / TICKS_PER_REV;
    }

    /** Core velocity control loop — call each loop */
    public void update() {
        if (masterMotor == null || followerMotor == null) return;

        double currentTPS = getMasterVelocityTPS();
        double error = targetTPS - currentTPS;

        // feedforward + proportional control → power output
        double power = kF * targetTPS + kP * error;
        power = Range.clip(power, 0.0, 1.0);

        masterMotor.setPower(power);
        followerMotor.setPower(power);
    }

    /** Stop both motors */
    public void stop() {
        if (masterMotor != null) masterMotor.setPower(0);
        if (followerMotor != null) followerMotor.setPower(0);
        targetRPM = 0;
        targetTPS = 0;
    }

    public void setCoefficients(double kP, double kF) {
        this.kP = kP;
        this.kF = kF;
    }
}
