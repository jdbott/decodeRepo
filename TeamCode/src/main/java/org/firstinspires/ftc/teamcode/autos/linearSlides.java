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

        waitForStart();

        for (int i = 0; i < 3; i++) {
            // Go up
            slides.goToPreset(offLinearSlides.HIGH);
            while (opModeIsActive() && slides.isBusy()) {
                slides.update();
                telemetry.addData("Error", slides.getError());
                telemetry.update();
            }
            sleep(200); // pause AFTER reaching target, inside the loop

            // Go down
            slides.goToPreset(offLinearSlides.RETRACTED);
            while (opModeIsActive() && slides.isBusy()) {  // fixed: isBusy(), not !isAtTarget()
                slides.update();
                telemetry.addData("Error", slides.getError());
                telemetry.update();
            }
            sleep(200); // pause after retraction too
        }

        while (opModeIsActive() && slides.isBusy()) {
            slides.update();
            telemetry.addData("pos", "%.2f", slides.getCurrentPositionInches());
            telemetry.addData("target", "%.2f", slides.getTargetPosition());
            telemetry.addData("error", "%.3f", slides.getError());
            telemetry.addData("busy", slides.isBusy());
            telemetry.update();
        }

        slides.stop();
    }
}