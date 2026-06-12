package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Feeder {

    private static final double ARM_BLOCK_POS = 0.28;
    private static final double ARM_SHOOT_POS = 0.42;
    private static final double CLUTCH_IN_POS = 0.48;
    private static final double CLUTCH_OUT_POS = 0.52;

    private final Servo armServo;
    private final Servo clutchServo;

    public Feeder(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, RobotConfig.FEEDER_ARM_SERVO);
        clutchServo = hardwareMap.get(Servo.class, RobotConfig.FEEDER_CLUTCH_SERVO);
    }

    public void armBlock() {
        armServo.setPosition(ARM_BLOCK_POS);
    }

    public void armShoot() {
        armServo.setPosition(ARM_SHOOT_POS);
    }

    public void clutchIn() {
        clutchServo.setPosition(CLUTCH_IN_POS);
    }

    public void clutchOut() {
        clutchServo.setPosition(CLUTCH_OUT_POS);
    }
}
