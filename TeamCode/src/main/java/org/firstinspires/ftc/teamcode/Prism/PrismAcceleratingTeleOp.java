package org.firstinspires.ftc.teamcode.Prism;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfig;

/**
 * PrismAcceleratingTeleOp — uses the exact same PrismAnimations classes
 * without modifying them, but ramps animation speed linearly from slow to fast
 * over 2.5 minutes (150 seconds).
 *
 * Linear ramp: speed(t) = START_SPEED + (END_SPEED - START_SPEED) * (t / DURATION)
 * where t is elapsed seconds since START, clamped to [0, DURATION].
 *
 * Uses a speed-based animation (Rainbow) whose 0..1 speed maps directly to
 * perceived velocity — so a linear increase in speed is a linear increase in
 * visual acceleration. For period-based animations the same idea applies with
 * period decreasing linearly (see comments).
 */
@TeleOp(name = "Prism Accelerating (2.5 min linear)", group = "Prism")
public class PrismAcceleratingTeleOp extends LinearOpMode {

    private static final double DURATION_SEC = 150.0; // 2.5 minutes
    private static final float START_SPEED = 0.05f;   // slow start
    private static final float END_SPEED = 1.0f;      // fast end

    // How often to push new speed over I2C (ms). 50ms = 20Hz is smooth
    // and well within the Prism's 400kHz I2C budget.
    private static final long UPDATE_INTERVAL_MS = 50;

    private GoBildaPrismDriver prism;
    private PrismAnimations.Rainbow rainbow;

    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, RobotConfig.PRISM_LED);

        // Exact same animation class from PrismAnimations.java — no modifications.
        rainbow = new PrismAnimations.Rainbow();
        rainbow.setStartHue(0.0f);
        rainbow.setStopHue(360.0f);
        rainbow.setDirection(Direction.Forward);
        rainbow.setRepeatAfter(25);
        rainbow.setBrightness(100);
        rainbow.setStartIndex(0);
        rainbow.setStopIndex(60);
        rainbow.setSpeed(START_SPEED);

        telemetry.addLine("Prism Accelerating — Linear ramp 2.5 min");
        telemetry.addData("Duration (s)", DURATION_SEC);
        telemetry.addData("START_SPEED", START_SPEED);
        telemetry.addData("END_SPEED", END_SPEED);
        telemetry.addData("Animation", "Rainbow (speed 0..1, exact same PrismAnimations.Rainbow)");
        telemetry.addData("Prism device", RobotConfig.PRISM_LED);
        telemetry.addLine("Press START — animation inserts then accelerates linearly.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        resetRuntime();

        // Insert once, then update speed in-place for the entire 2.5 min ramp.
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, rainbow);

        long lastUpdateMs = 0;

        while (opModeIsActive()) {
            double elapsedSec = getRuntime(); // seconds since resetRuntime()
            double fraction = Math.min(elapsedSec / DURATION_SEC, 1.0);
            float currentSpeed = (float) (START_SPEED + (END_SPEED - START_SPEED) * fraction);

            // Throttle I2C writes to UPDATE_INTERVAL_MS
            long nowMs = (long) (elapsedSec * 1000);
            if (nowMs - lastUpdateMs >= UPDATE_INTERVAL_MS) {
                rainbow.setSpeed(currentSpeed);
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_0);
                lastUpdateMs = nowMs;
            }

            telemetry.addData("Elapsed (s)", "%.1f / %.0f", elapsedSec, DURATION_SEC);
            telemetry.addData("Progress", "%.1f%%", fraction * 100.0);
            telemetry.addData("Speed (linear)", "%.3f", currentSpeed);
            telemetry.addData("Formula", "START + (END-START)*t/DURATION");
            if (fraction >= 1.0) {
                telemetry.addLine("Ramp complete — holding at END_SPEED.");
            }
            telemetry.update();

            // Keep loop responsive but not busy-waiting.
            sleep(20);

            // Optional: after 2.5 min hold at max speed until OpMode stops.
            // If you want to auto-stop at exactly 2.5 min, uncomment:
            // if (fraction >= 1.0) break;
        }
    }
}
