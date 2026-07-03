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

        // IF THE SLIDES GO THE WRONG WAY (down when told up), UNCOMMENT THIS:
        // slides.reverseMotorDirection();

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();

        for (int i = 0; i < 3; i++) {
            // Go up to 26"
            slides.goToPreset(offLinearSlides.HIGH);

            while (opModeIsActive() && slides.isBusy()) {
                telemetry.addData("Loop", i + 1);
                telemetry.addData("Direction", "UP");
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                slides.addTelemetry(telemetry);
                telemetry.update();
            }
            sleep(200);

            // Go down to 0"
            slides.goToPreset(offLinearSlides.RETRACTED);

            while (opModeIsActive() && slides.isBusy()) {
                telemetry.addData("Loop", i + 1);
                telemetry.addData("Direction", "DOWN");
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                slides.addTelemetry(telemetry);
                telemetry.update();
            }
            sleep(200);
        }

        slides.stop();
    }
}