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
        slides.setkS(0.15);

        // IF SLIDES GO THE WRONG WAY, UNCOMMENT THIS:
        // slides.setDirectionReversed(true);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();

        for (int i = 0; i < 3; i++) {
            // UP to 26"
            slides.goToPreset(offLinearSlides.HIGH);

            while (opModeIsActive() && slides.isBusy()) {
                slides.update();

                telemetry.addData("Loop", i + 1);
                telemetry.addData("Dir", "UP");
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                telemetry.addData("RawEnc", slides.getRawEncoder());
                telemetry.addData("Power", "%.3f", slides.getMotorPower());
                telemetry.update();
            }
            sleep(200);

            // DOWN to 0"
            slides.goToPreset(offLinearSlides.RETRACTED);

            while (opModeIsActive() && slides.isBusy()) {
                slides.update();

                telemetry.addData("Loop", i + 1);
                telemetry.addData("Dir", "DOWN");
                telemetry.addData("Target", "%.2f", slides.getTargetPosition());
                telemetry.addData("Current", "%.2f", slides.getCurrentPositionInches());
                telemetry.addData("Error", "%.3f", slides.getError());
                telemetry.addData("RawEnc", slides.getRawEncoder());
                telemetry.addData("Power", "%.3f", slides.getMotorPower());
                telemetry.update();
            }
            sleep(200);
        }

        slides.stop();
    }
}