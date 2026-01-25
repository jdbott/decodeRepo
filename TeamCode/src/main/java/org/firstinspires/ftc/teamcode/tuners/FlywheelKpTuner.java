package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardwareClasses.FlywheelASG;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Flywheel kP Tuner", group = "Flywheel Tuning")
public class FlywheelKpTuner extends LinearOpMode {

    private FlywheelASG flywheel;

    // Example target velocity (rad/s)
    private static final double TARGET_VELOCITY_RAD = 250;

    // Sweep parameters
    private static final double TEST_DURATION = 10000; // ms to measure response
    private static final int SAMPLE_RATE = 90; // ms per sample

    @Override
    public void runOpMode() {
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = new FlywheelASG(hardwareMap, battery);

        // Replace with feedforward values from previous tuner
        double kV = 0.0017;
        double kS = 0.1146;
        double tunedVoltage = 13.899;

        flywheel.setFeedforward(kV, kS, tunedVoltage);
        flywheel.setTargetVelocity(TARGET_VELOCITY_RAD);

        telemetry.addLine("kP Tuner Ready");
        telemetry.addLine("Measuring velocity response...");
        telemetry.update();

        waitForStart();

        List<Double> errors = new ArrayList<>();
        List<Double> velocities = new ArrayList<>();

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < TEST_DURATION) {
            flywheel.update();
            double currentVel = flywheel.getVelocityRadPerSec();
            double error = TARGET_VELOCITY_RAD - currentVel;

            errors.add(error);
            velocities.add(currentVel);

            telemetry.addData("Target (rad/s)", TARGET_VELOCITY_RAD);
            telemetry.addData("Velocity (rad/s)", currentVel);
            telemetry.addData("Error", error);
            telemetry.update();

            sleep(SAMPLE_RATE);
        }

        // Estimate ideal kP
        // Simplified: kP = max power needed to correct / max error
        double maxError = errors.stream().mapToDouble(Math::abs).max().orElse(1.0);
        double maxPower = 1.0 - (kV * TARGET_VELOCITY_RAD + kS) * tunedVoltage / battery.getVoltage();
        maxPower = Math.max(maxPower, 0.0);

        double estimatedKP = maxPower / maxError;

        telemetry.clearAll();
        telemetry.addLine("kP Tuning Complete");
        telemetry.addData("Estimated kP", estimatedKP);
        telemetry.addLine("Use in Flywheel class: add kP*error to feedforward power");
        telemetry.update();

        flywheel.stop();

        while (opModeIsActive()) idle();
    }
}