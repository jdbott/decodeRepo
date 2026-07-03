package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * MasterRobot - One class to rule them all.
 * Stores the HardwareMap and initializes every subsystem.
 *
 * WARNING: FTC requires UNIQUE device names in your RC config.
 * If you have multiple motors, you cannot actually name them all "cool".
 * Use unique names like "coolSlide", "coolDeposit", "coolLeft", "coolRight"
 * and update the strings below accordingly.
 */
public class offMasterRobot {

    /** Stored reference to the hardware map */
    public HardwareMap hardwareMap;

    // --- Subsystems ---
    public offLinearSlides slides;
    public offSeasonDeposit deposit;
    public offSeasonIntake intake;

    // --- Preset shortcuts ---
    public static final double SLIDES_RETRACTED = offLinearSlides.RETRACTED;
    public static final double SLIDES_LOW       = offLinearSlides.LOW;
    public static final double SLIDES_MID       = offLinearSlides.MID;
    public static final double SLIDES_HIGH      = offLinearSlides.HIGH;

    /**
     * Initializes all subsystems.
     * @param hwMap The hardwareMap from your OpMode
     */
    public offMasterRobot(HardwareMap hwMap) {
        this.hardwareMap = hwMap;

        // All motors use the name "cool" as requested
        slides  = new offLinearSlides(hwMap);           // default ctor uses "cool"
        deposit = new offSeasonDeposit(hwMap, "cool");  // pass "cool" explicitly
        intake  = new offSeasonIntake(hwMap);           // hardcoded to "cool" for both
    }

    /**
     * REMOVED: No longer needed. The new offLinearSlides uses RUN_TO_POSITION
     * which handles PID internally in the REV hub hardware at 1kHz.
     * You do NOT need to call update() every loop anymore.
     */
    // public void update() {
    //     slides.update();
    // }

    /**
     * Emergency stop for all subsystems.
     */
    public void stopAll() {
        slides.stop();
        deposit.stop();
        intake.stop();
    }
}