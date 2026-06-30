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
        slides.setMaxPower(0.8); // Optional: cap speed for smoother motion

        waitForStart();

        for (int i = 0; i < 3; i++) {
            // Go up
            slides.goToPreset(offLinearSlides.HIGH);
            while (opModeIsActive() && slides.isBusy()) {
                telemetry.addData("Loop", i + 1);
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                telemetry.update();
            }
            sleep(200);

            // Go down
            slides.goToPreset(offLinearSlides.RETRACTED);
            while (opModeIsActive() && slides.isBusy()) {
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