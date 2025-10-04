package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Flywheel Power Control")
public class FlywheelPowerControl extends LinearOpMode {

    // Shooter instance
    private Shooter shooter;

    // Power variables
    private double wheelVelocity = 0.0;
    private double wheelRPM = 0.0;
    private boolean useRPM = false;

    @Override
    public void runOpMode() {
        String[] motorNames = {"motor1", "motor2"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE};
        DcMotorEx.RunMode[] runModes = {DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, DcMotorEx.RunMode.RUN_USING_ENCODER};
        shooter = new Shooter();
        shooter.init(hardwareMap, motorNames, directions, runModes, "hoodServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Debounce control
        boolean dpadUpPressedLast = false;
        boolean dpadDownPressedLast = false;
        boolean aPressedLast = false;

        while (opModeIsActive()) {
            // Toggle control mode
            if (gamepad1.a && !aPressedLast) {
                useRPM = !useRPM;
            }
            aPressedLast = gamepad1.a;

            if (!useRPM) {
                // Velocity control
                if (gamepad1.dpad_up && !dpadUpPressedLast) {
                    wheelVelocity += 1.0; // Increase by 1 m/s
                    wheelVelocity = Range.clip(wheelVelocity, 0.0, 20.0);
                }
                if (gamepad1.dpad_down && !dpadDownPressedLast) {
                    wheelVelocity -= 1.0; // Decrease by 1 m/s
                    wheelVelocity = Range.clip(wheelVelocity, 0.0, 20.0);
                }
                shooter.spinMotors(wheelVelocity);
            } else {
                // RPM control
                if (gamepad1.dpad_up && !dpadUpPressedLast) {
                    wheelRPM += 100.0; // Increase by 100 RPM
                    wheelRPM = Range.clip(wheelRPM, 0.0, 6000.0);
                }
                if (gamepad1.dpad_down && !dpadDownPressedLast) {
                    wheelRPM -= 100.0; // Decrease by 100 RPM
                    wheelRPM = Range.clip(wheelRPM, 0.0, 6000.0);
                }
                shooter.spinMotorsRPM(wheelRPM);
            }

            //stop shooter
            if (gamepad1.dpad_left) {
                shooter.stopMotors();
                wheelVelocity = 0.0;
                wheelRPM = 0.0;
            }

            // Update last button states
            dpadUpPressedLast = gamepad1.dpad_up;
            dpadDownPressedLast = gamepad1.dpad_down;

            // Telemetry
            telemetry.addData("Control Mode", useRPM ? "RPM" : "Velocity (m/s)");
            if (!useRPM) {
                telemetry.addData("Target Velocity (m/s)", wheelVelocity);
                telemetry.addData("Current Velocity (m/s)", shooter.getCurrentVelocity());
            } else {
                telemetry.addData("Target RPM", wheelRPM);
                telemetry.addData("Current RPM", shooter.getCurrentVelocityRPM());
            }
            telemetry.addData("Switch Mode", "Press A");
            telemetry.update();
        }
        shooter.stopMotors();
    }
}