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
        slides.setMaxPower(1.0);              // MAX motor speed
        slides.setLimits(0.0, 26.0);          // Full height range
        slides.setProfileConstraints(35, 80); // Aggressive but smooth

        // If slides move the wrong way, uncomment this:
        // slides.reverseMotorDirection();

        // Static friction boost — needed to start moving from rest
        slides.setkS(0.15);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();

        // Run 3 cycles: up to 26in, down to 0in, 200ms between each
        for (int i = 0; i < 3; i++) {

            // --- GO UP to 26 inches ---
            slides.goToPreset(offLinearSlides.HIGH);

            while (opModeIsActive() && slides.isBusy()) {
                slides.update(); // MUST call every loop

                telemetry.addData("Loop", i + 1);
                telemetry.addData("Direction", "UP");
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                slides.addTelemetry(telemetry);
                telemetry.update();
            }
            sleep(200); // 200ms pause at top

            // --- GO DOWN to 0 inches ---
            slides.goToPreset(offLinearSlides.RETRACTED);

            while (opModeIsActive() && slides.isBusy()) {
                slides.update(); // MUST call every loop

                telemetry.addData("Loop", i + 1);
                telemetry.addData("Direction", "DOWN");
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                slides.addTelemetry(telemetry);
                telemetry.update();
            }
            sleep(200); // 200ms pause at bottom
        }

        slides.stop();
    }
}