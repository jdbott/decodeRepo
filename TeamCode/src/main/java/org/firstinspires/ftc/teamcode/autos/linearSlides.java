package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareClasses.offLinearSlides;

@Autonomous(name = "linear slides prototype", group = "Autonomous")
public class linearSlides extends LinearOpMode {

    private offLinearSlides slides;

    @Override
    public void runOpMode() {
        slides = new offLinearSlides(hardwareMap);
        slides.setMaxPower(1.0);              // MAX motor power
        slides.setLimits(0.0, 26.0);          // Allow full 26" height
        slides.setProfileConstraints(35, 80); // Max speed / accel

        // If you already ran the tuner and have values, paste them here:
        // slides.setPID(0.045, 0.012, 0.008);

        waitForStart();

        for (int i = 0; i < 3; i++) {
            // Go to FULL HEIGHT
            slides.goToPreset(offLinearSlides.HIGH);

            while (opModeIsActive() && slides.isBusy()) {
                slides.update(); // CRITICAL: must call every loop

                telemetry.addData("Loop", i + 1);
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                telemetry.addData("Vel", "%.2f", slides.getCurrentVelocityInches());
                telemetry.update();
            }
            sleep(200);

            // Go down
            slides.goToPreset(offLinearSlides.RETRACTED);

            while (opModeIsActive() && slides.isBusy()) {
                slides.update();

                telemetry.addData("Loop", i + 1);
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                telemetry.update();
            }
            sleep(200);
        }

        slides.stop();
    }
}