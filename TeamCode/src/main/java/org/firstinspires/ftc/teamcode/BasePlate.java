package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BasePlate {
    private static final double POPPER_DOWN = 0.07;
    private static final double POPPER_UP = 0.4;
    private static final double RAMP_BACK = 0.07;
    private static final double RAMP_FORWARD = 0.54;
    private final Servo popperFront;
    private final Servo popperMiddle;
    private final Servo ramp;

    private final Servo pusher;

    public BasePlate(HardwareMap hardwareMap) {
        popperFront = hardwareMap.get(Servo.class, "popperFront");
        popperMiddle = hardwareMap.get(Servo.class, "popperMiddle");
        ramp = hardwareMap.get(Servo.class, "ramp");
        pusher = hardwareMap.get(Servo.class, "pusherServo");
    }

    public void frontPopperUp() {
        popperFront.setPosition(POPPER_UP);
    }

    public void frontPopperDown() {
        popperFront.setPosition(POPPER_DOWN);
    }

    public void middlePopperUp() {
        popperMiddle.setPosition(POPPER_UP);
    }

    public void middlePopperDown() {
        popperMiddle.setPosition(POPPER_DOWN);
    }

    public void rampBack() {
        ramp.setPosition(RAMP_BACK);
    }

    public void rampForward() {
        ramp.setPosition(RAMP_FORWARD);
    }

    private static final double HOME_CMD = 0.41;

    private static final double SERVO_RANGE_DEG = 355.0;
    private static final double GT2_PITCH_MM = 2.0;
    private static final int CARRIAGE_PULLEY_TEETH = 77;
    private static final double SERVO_TO_PULLEY_RATIO = 60.0 / 20.0;

    private static final double MM_PER_COMMAND =
            (SERVO_RANGE_DEG / 360.0) *
                    SERVO_TO_PULLEY_RATIO *
                    (CARRIAGE_PULLEY_TEETH * GT2_PITCH_MM);

    private static final double MIN_MM = 0.0;
    private static final double MAX_MM = 150.0;

    private static final double CMD_MIN = 0.0;
    private static final double CMD_MAX = 1.0;

    public void setPusherCmd(double cmd) {
        if (cmd < CMD_MIN) cmd = CMD_MIN;
        if (cmd > CMD_MAX) cmd = CMD_MAX;
        pusher.setPosition(cmd);
    }

    public void setPusherMm(double mmFromHome) {
        if (mmFromHome < MIN_MM) mmFromHome = MIN_MM;
        if (mmFromHome > MAX_MM) mmFromHome = MAX_MM;

        double deltaCmd = mmFromHome / MM_PER_COMMAND;
        double cmd = HOME_CMD - deltaCmd;

        if (cmd < CMD_MIN) cmd = CMD_MIN;
        if (cmd > CMD_MAX) cmd = CMD_MAX;

        pusher.setPosition(cmd);
    }
}