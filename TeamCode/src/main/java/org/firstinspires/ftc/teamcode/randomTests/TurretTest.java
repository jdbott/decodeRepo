package org.firstinspires.ftc.teamcode.randomTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;

@Config
@TeleOp(name = "Turret Test")
public class TurretTest extends LinearOpMode {
    private Turret turret = new Turret();

    @Override
    public void runOpMode() {
        // Initialize turret
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.setKP(0.01);
        turret.setKF(0.003);

        telemetry.addLine("Turret + IMU Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Apply to turret
            if (gamepad1.dpad_up) {
                turret.setAngle(0);
            } else if (gamepad1.dpad_down) {
                turret.setAngle(180);
            } else if (gamepad1.dpad_left) {
                turret.setAngle(-45);
            } else if (gamepad1.dpad_right) {
                turret.setAngle(45);
            }
            turret.update();
        }
    }
}