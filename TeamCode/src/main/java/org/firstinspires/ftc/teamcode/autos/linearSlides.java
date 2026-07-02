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
        slides.setMaxPower(1.0);
        slides.setLimits(0.0, 26.0);
        slides.setProfileConstraints(35, 80);

        // If slides go the wrong way, uncomment this:
        // slides.reverseMotorDirection();

        slides.setkS(0.15);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();

        for (int i = 0; i < 3; i++) {
            // Go to FULL HEIGHT
            slides.goToPreset(offLinearSlides.HIGH);

            while (opModeIsActive() && slides.isBusy()) {
                slides.update();

                telemetry.addData("Loop", i + 1);
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                slides.addTelemetry(telemetry); // Shows Power, kP, kS, etc.
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
                slides.addTelemetry(telemetry);
                telemetry.update();
            }
            sleep(200);
        }

        slides.stop();
    }
}