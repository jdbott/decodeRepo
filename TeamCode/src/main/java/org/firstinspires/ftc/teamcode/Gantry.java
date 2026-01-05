package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gantry {
    private static final double FRONT_POS = 0.1;
    private static final double MIDDLE_POS = 0.36;
    private static final double BACK_POS = 0.75;

    private static String CURRENT_POS = "null";
    private final Servo servo1;
    private final Servo servo2;

    public Gantry(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(Servo.class, "gantry1");
        servo2 = hardwareMap.get(Servo.class, "gantry2");
    }

    public void moveGantryToPos(String pos) {
        switch (pos) {
            case "front":
                servo1.setPosition(FRONT_POS);
                servo2.setPosition(FRONT_POS);
                CURRENT_POS = "front";
                break;

            case "middle":
                servo1.setPosition(MIDDLE_POS);
                servo2.setPosition(MIDDLE_POS);
                CURRENT_POS = "middle";
                break;

            case "back":
                servo1.setPosition(BACK_POS);
                servo2.setPosition(BACK_POS);
                CURRENT_POS = "back";
                break;

            default:
                break;
        }
    }

    public String getGantryPos() {
        return CURRENT_POS;
    }
}
