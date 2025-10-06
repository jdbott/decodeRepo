package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "One motor shooter")
public class OneMotorShooterTest extends LinearOpMode {
    private OneMotorShooter shooter;

    @Override
    public void runOpMode() {
        shooter = new OneMotorShooter();
        shooter.init(hardwareMap, "turret", DcMotorEx.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                shooter.stopMotor();
            }

            if (gamepad1.dpad_right) {
                shooter.spinMotor(0.081);
            }

            if (gamepad1.dpad_up) {
                shooter.spinMotor(0.041);
            }

            if(gamepad1.dpad_left){
                shooter.spinMotorRPM(312);
            }

            if(gamepad1.left_bumper){
                shooter.spinMotorRPM(200);
            }

            telemetry.addData("Shooter Velocity (m/s)", shooter.getCurrentVelocity());
            telemetry.addData("Shooter Velocity (RPM)", shooter.getCurrentVelocityRPM());
            telemetry.update();
        }
    }
}
