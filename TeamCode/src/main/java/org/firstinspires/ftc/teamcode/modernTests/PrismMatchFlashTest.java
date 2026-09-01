package org.firstinspires.ftc.teamcode.modernTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.hardwareClasses.Prism;

@TeleOp(name = "Prism Match Flash Test", group = "Prism Testing")
public class PrismMatchFlashTest extends LinearOpMode {

    private Prism prism;

    // Whichever pattern is active starts slow and speeds up at a linear, constant pace
    // until time runs out. Set to 150.0 for a full 2:30 match; currently 15.0 for bench testing.
    private static final double MATCH_DURATION_SECONDS = 15.0;
    private static final float START_SPEED = 0.15f;
    private static final float END_SPEED = 1.0f;

    private static final Color PATTERN_COLOR = Color.BLUE;

    @Override
    public void runOpMode() {
        prism = new Prism(hardwareMap);

        telemetry.addLine("Prism Match Flash Test Ready");
        telemetry.addLine("Cross = Chase | Circle = Rainbow | Square = Pulse | Triangle = Blink");
        telemetry.update();

        waitForStart();
        resetRuntime();

        Prism.Pattern activePattern = Prism.Pattern.CHASE;
        prism.start(activePattern, PATTERN_COLOR, START_SPEED);

        while (opModeIsActive()) {
            // PS5 face buttons: Cross->a, Circle->b, Square->x, Triangle->y.
            if (gamepad1.aWasPressed()) {
                activePattern = Prism.Pattern.CHASE;
                prism.start(activePattern, PATTERN_COLOR, START_SPEED);
            } else if (gamepad1.bWasPressed()) {
                activePattern = Prism.Pattern.RAINBOW;
                prism.start(activePattern, PATTERN_COLOR, START_SPEED);
            } else if (gamepad1.xWasPressed()) {
                activePattern = Prism.Pattern.PULSE;
                prism.start(activePattern, PATTERN_COLOR, START_SPEED);
            } else if (gamepad1.yWasPressed()) {
                activePattern = Prism.Pattern.BLINK;
                prism.start(activePattern, PATTERN_COLOR, START_SPEED);
            }

            double elapsedSeconds = Range.clip(getRuntime(), 0.0, MATCH_DURATION_SECONDS);
            double progress = elapsedSeconds / MATCH_DURATION_SECONDS;
            float speed = (float) (START_SPEED + progress * (END_SPEED - START_SPEED));

            prism.setSpeed(speed);

            telemetry.addData("Pattern", activePattern);
            telemetry.addData("Match Time (s)", "%.1f", elapsedSeconds);
            telemetry.addData("Speed", speed);
            telemetry.update();

            sleep(50);
        }

        prism.turnOff();
    }
}
