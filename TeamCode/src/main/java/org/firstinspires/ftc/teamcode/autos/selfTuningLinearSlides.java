package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareClasses.offLinearSlides;

@Autonomous(name = "Tune Linear Slides", group = "Tuning")
public class selfTuningLinearSlides extends LinearOpMode {

    private offLinearSlides slides;

    @Override
    public void runOpMode() {
        slides = new offLinearSlides(hardwareMap);
        slides.setMaxPower(1.0);
        slides.setLimits(0.0, 26.0);
        slides.setProfileConstraints(40.0, 90.0); // aggressive for tuning

        telemetry.addLine("=== SLIDE AUTO-TUNER ===");
        telemetry.addLine("Make sure the slide area is clear!");
        telemetry.addLine("Press START to begin tuning...");
        telemetry.update();

        waitForStart();

        // Begin self-tuning
        slides.startAutoTune();

        while (opModeIsActive() && !slides.isTuningComplete()) {
            slides.update();

            telemetry.addLine("Tuning in progress... DO NOT touch the robot.");
            slides.addTelemetry(telemetry);
            telemetry.update();
        }

        // Tuning complete
        telemetry.addLine();
        telemetry.addLine("=== TUNING COMPLETE ===");
        telemetry.addLine("Copy this into your code:");
        telemetry.addLine(slides.getTunedGainsString());
        telemetry.addLine();
        telemetry.addLine("Press STOP to end.");
        slides.addTelemetry(telemetry);
        telemetry.update();

        // Hold telemetry on screen
        while (opModeIsActive()) {
            idle();
        }

        slides.stop();
    }
}