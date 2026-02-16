package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Gantry {
    // Base (mechanical) setpoints
    private static final double FRONT_POS  = 0.05;
    private static final double MIDDLE_POS = 0.6;
    private static final double BACK_POS   = 0.83;

    private String currentPosName = "back"; // avoid "null" so reapply works immediately

    private final Servo servo1;
    private final Servo servo2;

    // Runtime-only trim (clears each TeleOp run because the object is recreated)
    private double posOffset = 0.0;

    // Optional: limit how far the offset can go so you don't smash anything
    private static final double OFFSET_MIN = -0.25;
    private static final double OFFSET_MAX = +0.25;

    public Gantry(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(Servo.class, "gantry1");
        servo2 = hardwareMap.get(Servo.class, "gantry2");
    }

    // --- Public API ---

    public void moveGantryToPos(String pos) {
        switch (pos) {
            case "front":
                currentPosName = "front";
                setBoth(applyOffset(FRONT_POS));
                break;

            case "middle":
                currentPosName = "middle";
                setBoth(applyOffset(MIDDLE_POS));
                break;

            case "back":
                currentPosName = "back";
                setBoth(applyOffset(BACK_POS));
                break;

            default:
                // ignore
                break;
        }
    }

    /** Re-command the *current preset* (front/middle/back) using the current offset. */
    public void reapplyCurrentPreset() {
        switch (currentPosName) {
            case "front":
                setBoth(applyOffset(FRONT_POS));
                break;
            case "middle":
                setBoth(applyOffset(MIDDLE_POS));
                break;
            case "back":
            default:
                setBoth(applyOffset(BACK_POS));
                break;
        }
    }

    /** Nudges the *offset*, then physically moves the gantry by reapplying the current preset. */
    public void nudgeOffset(double delta) {
        posOffset += delta;

        // Clamp offset for safety
        posOffset = Range.clip(posOffset, OFFSET_MIN, OFFSET_MAX);

        // IMPORTANT: actually move to show the new offset
        reapplyCurrentPreset();
    }

    /** Treat the current physical servo position as "BACK_POS" for the rest of this TeleOp run. */
    public void zeroHereAsBack() {
        double current = getServoPosition(); // assume both are same
        posOffset = current - BACK_POS;

        // Clamp for safety
        posOffset = Range.clip(posOffset, OFFSET_MIN, OFFSET_MAX);

        // Lock logical preset to back AND reapply so it’s visible immediately
        currentPosName = "back";
        reapplyCurrentPreset();
    }

    /** Holds the current physical position (does NOT use presets/offset). */
    public void holdCurrent() {
        double cur = getServoPosition();
        setBoth(cur);
    }

    public String getGantryPosName() {
        return currentPosName;
    }

    public double getOffset() {
        return posOffset;
    }

    public double getServoPosition() {
        return servo1.getPosition();
    }

    // --- Helpers ---

    private double applyOffset(double base) {
        return Range.clip(base + posOffset, 0.0, 1.0);
    }

    private void setBoth(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        servo1.setPosition(pos);
        servo2.setPosition(pos);
    }
}