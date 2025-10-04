/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Shooter Test")
public class ShooterTest extends LinearOpMode {
    private Shooter shooter;

    @Override
    public void runOpMode() {
        String[] motorNames = {"shooter", "shooter2"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD};
        DcMotorEx.RunMode[] runModes = {DcMotorEx.RunMode.RUN_USING_ENCODER, DcMotorEx.RunMode.RUN_USING_ENCODER};
        shooter = new Shooter(hardwareMap, motorNames, directions, runModes, "hoodServo");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                shooter.stopMotors();
            }

            if (gamepad1.dpad_right) {
                shooter.spinMotors(5);
            }

            if (gamepad1.dpad_up) {
                shooter.spinMotors(10);
            }

            if(gamepad1.dpad_left){
                shooter.spinMotorsRPM(3000);
            }

            if (gamepad1.left_bumper) {
                shooter.setHoodAngle(30);
            }

            if (gamepad1.right_bumper) {
                shooter.setHoodAngle(60);
            }

            telemetry.addData("Shooter Velocity (m/s)", shooter.getCurrentVelocity());
            telemetry.addData("Shooter Velocity (RPM)", shooter.getCurrentVelocityRPM());
            telemetry.addData("Hood Position", shooter.hoodServo.getPosition());
            telemetry.update();
        }
    }
}
 */