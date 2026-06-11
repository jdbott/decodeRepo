package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {

    private static final double MIN_ANGLE_DEG = 30.0;
    private static final double MAX_ANGLE_DEG = 60.0;

    private static final double MIN_POS = 0.42;
    private static final double MAX_POS = 0.95;

    private final Servo hoodServo;

    public Hood(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    /** Sets the hood to the given launch angle (deg), clamped to range, and returns the clamped angle. */
    public double setAngle(double angleDeg) {
        double clampedAngleDeg = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, angleDeg));

        double t = (clampedAngleDeg - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
        double pos = MIN_POS + t * (MAX_POS - MIN_POS);
        pos = Math.max(MIN_POS, Math.min(MAX_POS, pos));

        hoodServo.setPosition(pos);

        return clampedAngleDeg;
    }
}
