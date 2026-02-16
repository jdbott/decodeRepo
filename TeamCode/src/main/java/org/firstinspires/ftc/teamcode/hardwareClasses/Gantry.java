package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Gantry {
    // Base (mechanical) setpoints
    private static final double FRONT_POS  = 0.05;
    private static final double MIDDLE_POS = 0.56;
    private static final double BACK_POS   = 0.83;

    private String currentPosName = "null";

    private final Servo servo1;
    private final Servo servo2;

    // Runtime-only trim (clears each TeleOp run because the object is recreated)
    private double posOffset = 0.0;

    public Gantry(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(Servo.class, "gantry1");
        servo2 = hardwareMap.get(Servo.class, "gantry2");
    }

    // --- Public API ---

    public void moveGantryToPos(String pos) {
        switch (pos) {
            case "front":
                setBoth(applyOffset(FRONT_POS));
                currentPosName = "front";
                break;

            case "middle":
                setBoth(applyOffset(MIDDLE_POS));
                currentPosName = "middle";
                break;

            case "back":
                setBoth(applyOffset(BACK_POS));
                currentPosName = "back";
                break;

            default:
                // ignore
                break;
        }
    }

    /** Nudges the *offset*, not the base positions. Positive = higher servo position. */
    public void nudgeOffset(double delta) {
        posOffset += delta;

        // Optional safety clamp: keep BACK within [0,1] (and thus the others likely safe too)
        double backWithOffset = BACK_POS + posOffset;
        if (backWithOffset > 1.0) posOffset -= (backWithOffset - 1.0);
        if (backWithOffset < 0.0) posOffset += (0.0 - backWithOffset);

        // Hold wherever we are right now while nudging
        holdCurrent();
    }

    /** Treat the current physical servo position as "BACK_POS" for the rest of this TeleOp run. */
    public void zeroHereAsBack() {
        double current = getServoPosition(); // assume both are same
        posOffset = current - BACK_POS;
        // keep it safe
        posOffset = Range.clip(posOffset, -1.0, 1.0);
        currentPosName = "back"; // logically you're aligning "back"
    }

    /** Actively command the current physical position again (useful while nudging). */
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