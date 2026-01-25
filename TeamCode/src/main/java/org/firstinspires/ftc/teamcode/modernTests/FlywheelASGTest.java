package org.firstinspires.ftc.teamcode.modernTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;

@TeleOp(name = "Flywheel Feedforward + kP Test", group = "Flywheel Testing")
public class FlywheelASGTest extends LinearOpMode {

    private FlywheelASG flywheel;

    // Example target velocity (rad/s)
    private double targetVelocityRad = 250;

    @Override
    public void runOpMode() {
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new FlywheelASG(hardwareMap, battery);

        // Set feedforward and kP
        flywheel.setTargetVelocity(targetVelocityRad);

        telemetry.addLine("Flywheel Feedforward + kP Test Ready");
        telemetry.addLine("Use gamepad right stick Y to adjust target velocity");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust target velocity with right stick
            targetVelocityRad += -gamepad1.right_stick_y * 5; // rad/s increment
            targetVelocityRad = Math.max(targetVelocityRad, 0); // no negative

            flywheel.setTargetVelocity(targetVelocityRad);
            flywheel.update();

            telemetry.addData("Target Velocity (rad/s)", targetVelocityRad);
            telemetry.addData("Actual Velocity (rad/s)", flywheel.getVelocityRadPerSec());
            telemetry.addData("Actual Velocity (RPM)", flywheel.getVelocityRPM());
            telemetry.addData("Applied Motor Power", flywheel.getPower());
            telemetry.update();
        }

        flywheel.stop();
    }
}