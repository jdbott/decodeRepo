package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BasePlate {
    private static final double POPPER_DOWN = 0.07;
    private static final double POPPER_UP = 0.4;
    private static final double RAMP_BACK = 0.07;
    private static final double RAMP_FORWARD = 0.615;

    private final Servo popperFront;
    private final Servo popperMiddle;
    private final Servo ramp;

    private final Servo pusher;
    private final Servo gate;

    public BasePlate(HardwareMap hardwareMap) {
        popperFront = hardwareMap.get(Servo.class, "popperFront");
        popperMiddle = hardwareMap.get(Servo.class, "popperMiddle");
        ramp = hardwareMap.get(Servo.class, "ramp");
        pusher = hardwareMap.get(Servo.class, "pusherServo");
        gate = hardwareMap.get(Servo.class, "gateServo");
    }

    public void frontPopperUp() { popperFront.setPosition(POPPER_UP); }
    public void frontPopperDown() { popperFront.setPosition(POPPER_DOWN); }

    public void middlePopperUp() { popperMiddle.setPosition(POPPER_UP); }
    public void middlePopperDown() { popperMiddle.setPosition(POPPER_DOWN); }

    public void rampBack() { ramp.setPosition(RAMP_BACK); }
    public void rampForward() { ramp.setPosition(RAMP_FORWARD); }

    // -----------------------
    // Pusher conversion
    // -----------------------
    private static final double HOME_CMD = 0.44;

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

    private double lastPusherMmCmd = 0.0;

    public void setPusherCmd(double cmd) {
        if (cmd < CMD_MIN) cmd = CMD_MIN;
        if (cmd > CMD_MAX) cmd = CMD_MAX;
        pusher.setPosition(cmd);
    }

    public void setPusherMm(double mmFromHome) {
        if (mmFromHome < MIN_MM) mmFromHome = MIN_MM;
        if (mmFromHome > MAX_MM) mmFromHome = MAX_MM;

        lastPusherMmCmd = mmFromHome;

        double deltaCmd = mmFromHome / MM_PER_COMMAND;
        double cmd = HOME_CMD - deltaCmd;

        if (cmd < CMD_MIN) cmd = CMD_MIN;
        if (cmd > CMD_MAX) cmd = CMD_MAX;

        pusher.setPosition(cmd);
    }

    // -----------------------
    // Gate positions
    // -----------------------
    public void gateUp() { gate.setPosition(0.8); }
    public void gateBackFullShoot() { gate.setPosition(0.1); }
    public void gateHoldBall1() { gate.setPosition(0.67); }
    public void gateHoldBall2() { gate.setPosition(0.6); }
    public void gateHoldBall3() { gate.setPosition(0.66); }
    public void gateHoldBall4() { gate.setPosition(0.25); }

    // -----------------------
    // Shooting FSM (split-start capable)
    // -----------------------
    private enum ShootState {
        IDLE,

        // "Prep" state: ramp forward + gate hold ball2, and STOP there.
        PREPPED,

        // Full/legacy start path
        RAMP_FORWARD_WAIT,
        DOWN_LITTLE_WAIT,

        // Mid/late execution
        PUSH_1_WAIT,
        PUSH_1_WAIT_MID,
        PUSH_1_WAIT_END,
        PUSH_2_AND_GATE_WAIT,
        RESET_RAMP_BACK_GATE_UP_WAIT,
        PUSHER_HOME_WAIT
    }

    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // Tunables
    private static final double SHOOT_PUSH_1_MM = 60;
    private static final double SHOOT_PUSH_2_MM = 25;

    private static final double DELAY_RAMP_FORWARD_S = 0.5;
    private static final double DELAY_DOWN_LITTLE_S = 0.2;
    private static final double DELAY_PUSH1_S = 0.2;
    private static final double DELAY_PUSH1_MID_S = 0.2;
    private static final double DELAY_PUSH1_END_S = 0.7;
    private static final double DELAY_PUSH2_AND_GATE_S = 0.5;
    private static final double DELAY_RESET_GATEUP_S = 0.2;
    private static final double DELAY_PUSHER_HOME_S = 0.2;

    // -----------------------
    // Public entry points you asked for
    // -----------------------

    // 1) PREP ONLY: ramp forward + gate hold ball2, then stop.
    //    No pusher movement.
    public void prepShootOnly() {
        if (shootState != ShootState.IDLE) return;

        rampForward();
        gateHoldBall2();
        shootTimer.reset();
        shootState = ShootState.PREPPED;
    }

    // 2) From PREP: immediately move pusher to SHOOT_PUSH_1_MM, then run the rest.
    //    (This skips the earlier ramp-forward wait + down-little wait.)
    public void startShootFromPrep() {
        if (shootState != ShootState.PREPPED) return;

        setPusherMm(SHOOT_PUSH_1_MM);
        shootTimer.reset();
        shootState = ShootState.PUSH_1_WAIT;
    }

    // 3) Start at PUSH_1_WAIT directly (late start).
    //    Assumes you already have ramp forward + gate positions where you want them,
    //    and (typically) pusher already at SHOOT_PUSH_1_MM.
    public void startShootFromPush1Wait() {
        if (shootState != ShootState.IDLE && shootState != ShootState.PREPPED) return;

        rampForward();
        gateHoldBall2();
        shootTimer.reset();
        shootState = ShootState.PUSH_1_WAIT;
    }

    // Optional convenience: do the original “one-call full sequence” start.
    public void startFullShoot() {
        if (shootState != ShootState.IDLE) return;

        rampForward();
        gateHoldBall1();
        shootTimer.reset();
        shootState = ShootState.RAMP_FORWARD_WAIT;
    }

    public boolean isShootBusy() {
        return shootState != ShootState.IDLE;
    }

    public boolean isPrepped() {
        return shootState == ShootState.PREPPED;
    }

    public void cancelShootAndReset() {
        shootState = ShootState.IDLE;

        rampBack();
        gateUp();
        setPusherMm(0.0);
    }

    // Call every loop
    public void update() {
        switch (shootState) {
            case IDLE:
                return;

            case PREPPED:
                return;

            case RAMP_FORWARD_WAIT:
                if (shootTimer.seconds() >= DELAY_RAMP_FORWARD_S) {
                    gateHoldBall2();
                    shootTimer.reset();
                    shootState = ShootState.DOWN_LITTLE_WAIT;
                }
                break;

            case DOWN_LITTLE_WAIT:
                if (shootTimer.seconds() >= DELAY_DOWN_LITTLE_S) {
                    setPusherMm(SHOOT_PUSH_1_MM);
                    shootTimer.reset();
                    shootState = ShootState.PUSH_1_WAIT;
                }
                break;

            case PUSH_1_WAIT:
                if (shootTimer.seconds() >= DELAY_PUSH1_S) {
                    setPusherMm(SHOOT_PUSH_1_MM + SHOOT_PUSH_2_MM);
                    gateHoldBall3();
                    shootTimer.reset();
                    shootState = ShootState.PUSH_1_WAIT_MID;
                }
                break;

            case PUSH_1_WAIT_MID:
                if (shootTimer.seconds() >= DELAY_PUSH1_MID_S) {
                    gateHoldBall4();
                    shootTimer.reset();
                    shootState = ShootState.PUSH_1_WAIT_END;
                }
                break;

            case PUSH_1_WAIT_END:
                if (shootTimer.seconds() >= DELAY_PUSH1_END_S) {
                    setPusherMm(SHOOT_PUSH_1_MM + SHOOT_PUSH_2_MM - 40);
                    gateBackFullShoot();
                    shootTimer.reset();
                    shootState = ShootState.PUSH_2_AND_GATE_WAIT;
                }
                break;

            case PUSH_2_AND_GATE_WAIT:
                if (shootTimer.seconds() >= DELAY_PUSH2_AND_GATE_S) {
                    rampBack();
                    gateUp();
                    shootTimer.reset();
                    shootState = ShootState.RESET_RAMP_BACK_GATE_UP_WAIT;
                }
                break;

            case RESET_RAMP_BACK_GATE_UP_WAIT:
                if (shootTimer.seconds() >= DELAY_RESET_GATEUP_S) {
                    setPusherMm(0.0);
                    shootTimer.reset();
                    shootState = ShootState.PUSHER_HOME_WAIT;
                }
                break;

            case PUSHER_HOME_WAIT:
                if (shootTimer.seconds() >= DELAY_PUSHER_HOME_S) {
                    shootState = ShootState.IDLE;
                }
                break;

            default:
                shootState = ShootState.IDLE;
                break;
        }
    }
}