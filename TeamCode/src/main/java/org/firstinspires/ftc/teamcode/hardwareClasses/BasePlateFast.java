package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BasePlateFast {
    private static final double POPPER_DOWN = 0.076;
    private static final double POPPER_UP = 0.35;
    private static final double RAMP_BACK = 0.07;
    private static final double RAMP_FORWARD = 0.615;

    private final Servo popperFront;
    private final Servo popperMiddle;
    private final Servo ramp;

    private final Servo pusher;
    private final Servo gate;

    public BasePlateFast(HardwareMap hardwareMap) {
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
    private static final double HOME_CMD = 0.4;

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

    public double getLastPusherMmCmd() {
        return lastPusherMmCmd;
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

    public void gateRotateBack() { gateHoldBall2(); }
    public void gateRotateStage2() { gateHoldBall3(); }
    public void gateRotateStage3() { gateHoldBall4(); }
    public void gateRotateShoot() { gateBackFullShoot(); }

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
        PUSHER_HOME_WAIT,

        TEST
    }

    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // Tunables
    private static final double SHOOT_PUSH_1_MM = 60;
    private static final double SHOOT_PUSH_2_MM = 30;

    private static final double DELAY_RAMP_FORWARD_S = 0.5;
    private static final double DELAY_DOWN_LITTLE_S = 0.2;
    private static final double DELAY_PUSH1_S = 0.1;
    private static final double DELAY_PUSH1_MID_S = 0.1;
    private static final double DELAY_PUSH1_END_S = 0.5;
    private static final double DELAY_PUSH2_AND_GATE_S = 0.5;
    private static final double DELAY_RESET_GATEUP_S = 0.2;
    private static final double DELAY_PUSHER_HOME_S = 0.2;

    // ============================================================
    // Inter-shot spacing (AUTO-only feature; TeleOp leaves at 0)
    // ============================================================

    // Extra required delay between shot 1->2 and shot 2->3
    private double extraDelay12S = 0.0;
    private double extraDelay23S = 0.0;

    // Timer that measures time since the last "shot event"
    private final ElapsedTime interShotTimer = new ElapsedTime();

    // Which spacing we are waiting for next: 0=none, 1=waiting before shot2, 2=waiting before shot3
    private int nextSpacingIndex = 0;

    /** Set explicit extra delays (seconds) between shot1->shot2 and shot2->shot3. */
    public void setInterShotExtraDelays(double delay12S, double delay23S) {
        this.extraDelay12S = Math.max(0.0, delay12S);
        this.extraDelay23S = Math.max(0.0, delay23S);
    }

    /**
     * Convenience: compute extra delays based on ball colors.
     * If color changes between i and i+1, apply delayColorChangeS; otherwise 0.
     */
    public void setInterShotDelaysFromColors(char c1, char c2, char c3, double delayColorChangeS) {
        double d = Math.max(0.0, delayColorChangeS);
        this.extraDelay12S = (c1 == c2) ? 0.0 : d;
        this.extraDelay23S = (c2 == c3) ? 0.0 : d;
    }

    /** Returns the active required delay for the next boundary we're about to enforce. */
    private double getRequiredExtraDelayForNextBoundary() {
        if (nextSpacingIndex == 1) return extraDelay12S;
        if (nextSpacingIndex == 2) return extraDelay23S;
        return 0.0;
    }

    /** Call when a shot happens so we start timing spacing until the next shot. */
    private void markShotFired() {
        interShotTimer.reset();
    }

    /** True if we've satisfied the extra spacing requirement for the next boundary. */
    private boolean spacingSatisfied() {
        double req = getRequiredExtraDelayForNextBoundary();
        return interShotTimer.seconds() >= req;
    }

    // Small settle time after meeting spacing (optional but nice)
    private static final double HOLD_SETTLE_S = 0.0;

    // -----------------------
    // Existing getters (unchanged signatures)
    // -----------------------
    public double getShootPush1Mm() { return SHOOT_PUSH_1_MM; }
    public double getShootPush2Mm() { return SHOOT_PUSH_2_MM; }

    public long getDelayPush1Ms() { return secondsToMs(DELAY_PUSH1_S); }
    public long getDelayPush1MidMs() { return secondsToMs(DELAY_PUSH1_MID_S); }
    public long getDelayPush1EndMs() { return secondsToMs(DELAY_PUSH1_END_S); }
    public long getDelayGateRotateMs() { return secondsToMs(DELAY_DOWN_LITTLE_S); }
    public long getDelayResetGateUpMs() { return secondsToMs(DELAY_RESET_GATEUP_S); }
    public long getDelayRotateBackMs() { return secondsToMs(DELAY_PUSH1_MID_S); }
    public long getDelayRotateResetMs() { return secondsToMs(DELAY_PUSHER_HOME_S); }

    private long secondsToMs(double seconds) {
        return (long) (seconds * 1000.0);
    }

    // -----------------------
    // Public entry points (unchanged names)
    // -----------------------

    // 1) PREP ONLY: ramp forward + gate hold ball2, then stop. No pusher movement.
    public void prepShootOnly() {
        if (shootState != ShootState.IDLE) return;

        rampForward();
        gateHoldBall2();

        // spacing bookkeeping
        nextSpacingIndex = 0;

        shootTimer.reset();
        shootState = ShootState.PREPPED;
    }

    // 2) From PREP: immediately move pusher to SHOOT_PUSH_1_MM, then run the rest.
    public void startShootFromPrep() {
        if (shootState != ShootState.PREPPED) return;

        setPusherMm(SHOOT_PUSH_1_MM);

        // Shot 1 begins now
        markShotFired();
        nextSpacingIndex = 1; // next boundary is "before shot 2"

        shootTimer.reset();
        shootState = ShootState.PUSH_1_WAIT;
    }

    // 3) Start at PUSH_1_WAIT directly (late start).
    public void startShootFromPush1Wait() {
        if (shootState != ShootState.IDLE && shootState != ShootState.PREPPED) return;

        rampForward();
        gateHoldBall2();

        // conservative: treat as shot 1 starting now
        markShotFired();
        nextSpacingIndex = 1;

        shootTimer.reset();
        shootState = ShootState.PUSH_1_WAIT;
    }

    // Optional convenience: original “one-call full sequence” start.
    public void startFullShoot() {
        if (shootState != ShootState.IDLE) return;

        rampForward();
        gateHoldBall1();

        nextSpacingIndex = 0;

        shootTimer.reset();
        shootState = ShootState.RAMP_FORWARD_WAIT;
    }

    public boolean isShootBusy() {
        return shootState != ShootState.IDLE;
    }

    public boolean isPrepped() {
        return shootState == ShootState.PREPPED;
    }

    public ShootState getShootState() {
        return shootState;
    }

    /** True when FSM is currently being held by inter-shot spacing logic. */
    public boolean isHoldingForInterShotSpacing() {
        if (nextSpacingIndex == 1) {
            // Holding happens right before we command gateHoldBall4 (shot #2 moment),
            // which is inside PUSH_1_WAIT_MID.
            return shootState == ShootState.PUSH_1_WAIT_MID && !spacingSatisfied();
        }
        if (nextSpacingIndex == 2) {
            // Holding happens right before we command gateBackFullShoot (shot #3 moment),
            // which is inside TEST.
            return shootState == ShootState.TEST && !spacingSatisfied();
        }
        return false;
    }

    public void cancelShootAndReset() {
        shootState = ShootState.IDLE;

        rampBack();
        gateUp();
        setPusherMm(0.0);

        // reset spacing bookkeeping (safe default for tests)
        nextSpacingIndex = 0;
        extraDelay12S = 0.0;
        extraDelay23S = 0.0;
    }

    // 4) Start at the point "right before firing ball 2"
// Assumes the caller already staged:
//   - rampForward()
//   - gateHoldBall3()
//   - pusher at (SHOOT_PUSH_1_MM + SHOOT_PUSH_2_MM)
    public void startShootFromBeforeShot2() {
        // Allow starting from IDLE or PREPPED (don’t start if already in the middle of a shoot)
        if (shootState != ShootState.IDLE && shootState != ShootState.PREPPED) return;

        // Treat as if "shot 1 happened already" for spacing bookkeeping.
        // If delay12 is set to 0 (recommended for this entry), spacingSatisfied() will be immediate.
        markShotFired();
        nextSpacingIndex = 1; // "before shot2" boundary (will be satisfied immediately if delay12=0)

        shootTimer.reset();
        shootState = ShootState.PUSH_1_WAIT_MID;
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

                    // Shot 1 begins now (full-start path)
                    markShotFired();
                    nextSpacingIndex = 1;

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

                    // Shot #2 moment
                    gateBackFullShoot();

                    // Now measure spacing until shot 3
                    markShotFired();
                    nextSpacingIndex = 2;

                    shootTimer.reset();
                    shootState = ShootState.PUSH_1_WAIT_END;
                }
                break;

            case PUSH_1_WAIT_END:
                if (shootTimer.seconds() >= DELAY_PUSH1_END_S) {
                    setPusherMm(SHOOT_PUSH_1_MM + SHOOT_PUSH_2_MM - 40);
                    gateHoldBall2();
                    shootTimer.reset();
                    shootState = ShootState.TEST;
                }
                break;

            case TEST:
                if (shootTimer.seconds() >= 0.2) {

                    // BEFORE shot #3 moment: enforce extra spacing from shot #2.
                    if (!spacingSatisfied()) {
                        break;
                    }

                    if (HOLD_SETTLE_S > 0.0 && interShotTimer.seconds() < getRequiredExtraDelayForNextBoundary() + HOLD_SETTLE_S) {
                        break;
                    }

                    // Shot #3 moment
                    gateBackFullShoot();

                    // No further spacing
                    markShotFired();
                    nextSpacingIndex = 0;

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