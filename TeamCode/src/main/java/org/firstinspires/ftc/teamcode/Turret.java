package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    private final DcMotorEx turretMotor;
    private final double ticksPerDegree;
    private final double maxAngle;
    private final double minAngle;
    private double lastSetAngle = 0;
    private boolean turretBusy = false;
    private double kP = 0.01;

    public Turret(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction,
                  double ticksPerRevolution, double gearReduction, double minAngle, double maxAngle) {
        this.ticksPerDegree = (ticksPerRevolution * gearReduction) / 360.0;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turretMotor.setDirection(direction);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void zeroTurret() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastSetAngle = 0;
        rotateToAngle(0);
    }

    private double angleToTicks(double angle) {
        return angle * ticksPerDegree;
    }

    private double ticksToAngle(double ticks) {
        return ticks / ticksPerDegree;
    }

    public void rotateToAngle(double targetAngle) {
        // Clamp target angle within allowed range
        targetAngle = Range.clip(targetAngle, minAngle, maxAngle);
        double targetTicks = angleToTicks(targetAngle);
        moveTurretToPosition(targetTicks);
    }

    private void moveTurretToPosition(double targetTicks) {
        lastSetAngle = ticksToAngle(targetTicks);
        double error = targetTicks - turretMotor.getCurrentPosition();
        if (Math.abs(error) > 10) { // error threshold
            double power = error * kP;
            turretMotor.setPower(Range.clip(power, -1, 1));
            turretBusy = true;
        } else {
            turretMotor.setPower(0);
            turretBusy = false;
        }
    }

    public void correctTurretPosition() {
        if (!turretBusy) {
            double error = angleToTicks(lastSetAngle) - turretMotor.getCurrentPosition();
            turretMotor.setPower(Range.clip((error * kP), -1, 1));
        }
    }

    public void update() {
        if (isTurretBusy()) {
            moveTurretToPosition(angleToTicks(lastSetAngle));
        } else {
            correctTurretPosition();
        }
    }

    public boolean isTurretBusy() {
        return turretBusy;
    }

    public double getCurrentAngle() {
        return ticksToAngle(turretMotor.getCurrentPosition());
    }
}
