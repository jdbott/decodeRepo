package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.Direction;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.RobotConfig;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

/**
 * Wraps the goBILDA Prism RGB LED driver (see {@code org.firstinspires.ftc.teamcode.Prism})
 * as a constructor-injected hardware class following this repo's subsystem pattern.
 */
public class Prism {

    public enum Pattern { CHASE, RAINBOW, PULSE, BLINK }

    // Every one-cycle-length bounds a normalized 0..1 speed value gets mapped onto (shorter =
    // faster). Also used to throttle updates: a pattern's descriptor is only rewritten once its
    // own current cycle has had time to finish, instead of on a fixed clock. A fixed interval
    // either cuts off slow cycles early (choppy) or re-sends the same value many times per cycle
    // once the pattern is already fast (wasted, and can still stutter it), so the interval
    // between updates has to track the pattern's own speed, not a constant.
    private static final int CHASE_MAX_PERIOD_MS = 1200;
    private static final int CHASE_MIN_PERIOD_MS = 150;
    private static final int RAINBOW_MAX_PERIOD_MS = 1200;
    private static final int RAINBOW_MIN_PERIOD_MS = 150;
    private static final int PULSE_MAX_PERIOD_MS = 1500;
    private static final int PULSE_MIN_PERIOD_MS = 150;
    private static final int BLINK_MAX_PERIOD_MS = 1000;
    private static final int BLINK_MIN_PERIOD_MS = 120;

    private final GoBildaPrismDriver driver;
    private final PrismAnimations.Snakes chase = new PrismAnimations.Snakes();
    private final PrismAnimations.Rainbow rainbow = new PrismAnimations.Rainbow();
    private final PrismAnimations.Pulse pulse = new PrismAnimations.Pulse();
    private final PrismAnimations.Blink blink = new PrismAnimations.Blink();

    // Total LEDs actually wired on the strip(s), read from the device itself. Several vendor
    // animation defaults (e.g. Snakes' repeatAfter/snakeLength) are sized for a much longer
    // virtual strip (255px) than goBilda's 12-LED/200mm segments, so left unset a pattern only
    // ever plays out over a sliver of the real strip.
    private final int numberOfLeds;

    private Pattern activePattern;
    private long nextUpdateAllowedMillis = 0;

    public Prism(HardwareMap hardwareMap) {
        this(hardwareMap, RobotConfig.PRISM_LED);
    }

    public Prism(HardwareMap hardwareMap, String deviceName) {
        driver = hardwareMap.get(GoBildaPrismDriver.class, deviceName);
        numberOfLeds = driver.getNumberOfLEDs();
    }

    /**
     * Start (or restart) the given pattern in {@code color} at the given speed.
     * Direction.Forward reads as left-to-right on this robot's strip orientation for CHASE
     * and RAINBOW; flip to Direction.Backward in the code below if the wiring runs the other way.
     *
     * @param speed how fast the pattern moves, from 0 (slow) to 1 (fast).
     */
    public void start(Pattern pattern, Color color, float speed) {
        activePattern = pattern;
        int cyclePeriodMs;
        switch (pattern) {
            case CHASE:
                cyclePeriodMs = startChase(color, speed);
                break;
            case RAINBOW:
                cyclePeriodMs = startRainbow(speed);
                break;
            case PULSE:
                cyclePeriodMs = startPulse(color, speed);
                break;
            case BLINK:
            default:
                cyclePeriodMs = startBlink(color, speed);
                break;
        }
        nextUpdateAllowedMillis = System.currentTimeMillis() + cyclePeriodMs;
    }

    /**
     * Update the speed (0 to 1) of whichever pattern is currently active. Safe to call every
     * loop — internally gated so the descriptor is only rewritten once the pattern's current
     * cycle (at its current speed) has had time to actually finish playing.
     */
    public void setSpeed(float speed) {
        if (activePattern == null)
            return;

        long now = System.currentTimeMillis();
        if (now < nextUpdateAllowedMillis)
            return;

        int cyclePeriodMs;
        switch (activePattern) {
            case CHASE:
                cyclePeriodMs = speedToPeriodMs(speed, CHASE_MAX_PERIOD_MS, CHASE_MIN_PERIOD_MS);
                chase.setSpeed(speed);
                break;
            case RAINBOW:
                cyclePeriodMs = speedToPeriodMs(speed, RAINBOW_MAX_PERIOD_MS, RAINBOW_MIN_PERIOD_MS);
                rainbow.setSpeed(speed);
                break;
            case PULSE:
                cyclePeriodMs = speedToPeriodMs(speed, PULSE_MAX_PERIOD_MS, PULSE_MIN_PERIOD_MS);
                pulse.setPeriod(cyclePeriodMs);
                break;
            case BLINK:
            default:
                cyclePeriodMs = speedToPeriodMs(speed, BLINK_MAX_PERIOD_MS, BLINK_MIN_PERIOD_MS);
                blink.setPeriod(cyclePeriodMs);
                blink.setPrimaryColorPeriod(cyclePeriodMs / 2);
                break;
        }
        driver.updateAnimationFromIndex(LayerHeight.LAYER_0);
        nextUpdateAllowedMillis = now + cyclePeriodMs;
    }

    /** Turn off all animations/LEDs. */
    public void turnOff() {
        driver.clearAllAnimations();
        activePattern = null;
    }

    /** Escape hatch for anything not wrapped above (custom animations, artboards, telemetry). */
    public GoBildaPrismDriver getDriver() {
        return driver;
    }

    private int startChase(Color color, float speed) {
        int snakeLength = Math.max(1, numberOfLeds / 4);

        chase.setColors(color);
        chase.setBackgroundColor(Color.TRANSPARENT);
        chase.setDirection(Direction.Forward);
        chase.setSpeed(speed);
        chase.setSnakeLength(snakeLength);
        chase.setSpacingBetween(snakeLength);
        chase.setRepeatAfter(numberOfLeds);
        chase.setIndexes(0, Math.max(0, numberOfLeds - 1));
        driver.insertAndUpdateAnimation(LayerHeight.LAYER_0, chase);
        return speedToPeriodMs(speed, CHASE_MAX_PERIOD_MS, CHASE_MIN_PERIOD_MS);
    }

    private int startRainbow(float speed) {
        rainbow.setHues(0f, 360f);
        rainbow.setDirection(Direction.Forward);
        rainbow.setSpeed(speed);
        rainbow.setRepeatAfter(numberOfLeds);
        driver.insertAndUpdateAnimation(LayerHeight.LAYER_0, rainbow);
        return speedToPeriodMs(speed, RAINBOW_MAX_PERIOD_MS, RAINBOW_MIN_PERIOD_MS);
    }

    private int startPulse(Color color, float speed) {
        int periodMs = speedToPeriodMs(speed, PULSE_MAX_PERIOD_MS, PULSE_MIN_PERIOD_MS);

        pulse.setPrimaryColor(color);
        pulse.setSecondaryColor(Color.TRANSPARENT);
        pulse.setPeriod(periodMs);
        driver.insertAndUpdateAnimation(LayerHeight.LAYER_0, pulse);
        return periodMs;
    }

    private int startBlink(Color color, float speed) {
        int periodMs = speedToPeriodMs(speed, BLINK_MAX_PERIOD_MS, BLINK_MIN_PERIOD_MS);

        blink.setPrimaryColor(color);
        blink.setSecondaryColor(Color.TRANSPARENT);
        blink.setPeriod(periodMs);
        blink.setPrimaryColorPeriod(periodMs / 2);
        driver.insertAndUpdateAnimation(LayerHeight.LAYER_0, blink);
        return periodMs;
    }

    private int speedToPeriodMs(float speed, int maxPeriodMs, int minPeriodMs) {
        float clipped = Range.clip(speed, 0f, 1f);
        return (int) (maxPeriodMs - clipped * (maxPeriodMs - minPeriodMs));
    }
}
