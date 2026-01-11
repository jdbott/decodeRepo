package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter velocity controller with:
 *  - Full-power spinup until near target
 *  - PIDF hold once near target
 *  - "Ready" gating (within tolerance for hold time)
 *
 * This is a refactor of ServoZeroPopper's shooter control portion into a hardware class format.
 * Shot detection logic has been intentionally removed.
 */
public class ShooterV2 {

    // Hardware
    private DcMotorEx shootMotor1; // mechanically linked
    private DcMotorEx shootMotor2; // encoder motor (velocity source) OR vice versa based on your wiring

    // ----------------------------
    // Constants / configuration
    // ----------------------------
    private static final double TICKS_PER_REV = 28.0;

    // Target velocity in ticks/sec
    private double targetTPS = 0.0;

    // Ready-to-shoot tolerance (1%) and hold time
    private static final double READY_TOLERANCE_FRAC = 0.01;
    private static final long READY_HOLD_MS = 80;

    // --- Fast spin-up behavior ---
    private static final double SPINUP_ENTER_FRAC = 0.92;
    private static final double SPINUP_EXIT_FRAC  = 0.88;

    // --- PIDF hold gains ---
    // Matches your opmode values
    private double kF = 0.00058;
    private double kP = 0.00030;
    private double kD = 0.00008;
    private double kI = 0.00012;

    // Anti-windup clamp on integral accumulator (tick-sec)
    private static final double I_CLAMP = 100.0;

    private static final double MIN_POWER = 0.0;
    private static final double MAX_POWER = 1.0;

    // ----------------------------
    // Runtime state
    // ----------------------------
    private boolean enabled = false;
    private boolean spinupMode = true;

    private double lastError = 0.0;
    private double iSum = 0.0;
    private long lastTimeNs = 0;

    // Ready gating state
    private long readySinceMs = 0;
    private boolean readyStable = false;

    // Telemetry / debug fields (optional)
    private double lastCmdPower = 0.0;
    private double lastFF = 0.0;
    private double lastPTerm = 0.0;
    private double lastITerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastVelTPS = 0.0;
    private double lastAbsErrPct = 0.0;

    /**
     * Initialize shooter motors.
     *
     * @param hw hardware map
     * @param motor1Name first shooter motor name
     * @param motor2Name second shooter motor name
     * @param motor1Dir direction for motor1
     * @param motor2Dir direction for motor2
     */
    public void init(HardwareMap hw,
                     String motor1Name, String motor2Name,
                     DcMotorSimple.Direction motor1Dir, DcMotorSimple.Direction motor2Dir) {

        shootMotor1 = hw.get(DcMotorEx.class, motor1Name);
        shootMotor2 = hw.get(DcMotorEx.class, motor2Name);

        shootMotor1.setDirection(motor1Dir);
        shootMotor2.setDirection(motor2Dir);

        shootMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetControllerState();
        stop(); // ensure motors are off initially
    }

    // ----------------------------
    // Public API (similar spirit to your Shooter class)
    // ----------------------------

    /** Enable/disable shooter control (like shooterOn toggle). */
    public void setEnabled(boolean enabled) {
        if (this.enabled != enabled) {
            this.enabled = enabled;
            resetControllerState();
            if (!enabled) {
                setMotorPower(0.0);
            }
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Set desired velocity in ticks/sec (same variable concept as old code). */
    public void setTargetTPS(double ticksPerSec) {
        targetTPS = Math.max(0.0, ticksPerSec);
        // when changing target while enabled, it’s typically safest to re-enter spinup
        if (enabled) {
            spinupMode = true;
            readySinceMs = 0;
            readyStable = false;
            iSum = 0.0;
        }
    }

    /** Convenience: set desired velocity in RPM. */
    public void setTargetRPM(double rpm) {
        setTargetTPS(rpm * TICKS_PER_REV / 60.0);
    }

    public double getTargetTPS() { return targetTPS; }
    public double getTargetRPM() { return targetTPS * 60.0 / TICKS_PER_REV; }

    /** Velocity source (matches your opmode: shootMotor2.getVelocity()). */
    public double getVelocityTPS() {
        if (shootMotor2 == null) return 0.0;
        return shootMotor2.getVelocity();
    }

    public double getVelocityRPM() {
        return getVelocityTPS() * 60.0 / TICKS_PER_REV;
    }

    /** True when within tolerance for READY_HOLD_MS and not in spinup mode. */
    public boolean isReady() {
        return readyStable;
    }

    /** Main control loop: call every loop. */
    public void update() {
        if (!enabled || shootMotor1 == null || shootMotor2 == null) {
            readyStable = false;
            return;
        }

        final double vel = getVelocityTPS();
        lastVelTPS = vel;

        // Unsigned percent error for readiness + debug
        double errorNow = targetTPS - vel;
        double errFrac = (targetTPS <= 1.0) ? 0.0 : (Math.abs(errorNow) / targetTPS);
        lastAbsErrPct = errFrac * 100.0;

        // Timing
        long nowNs = System.nanoTime();
        if (lastTimeNs == 0) lastTimeNs = nowNs;
        double dt = (nowNs - lastTimeNs) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastTimeNs = nowNs;

        // Decide spinup vs hold
        double frac = (targetTPS <= 1.0) ? 0.0 : (vel / targetTPS);

        if (spinupMode) {
            if (frac >= SPINUP_ENTER_FRAC) {
                spinupMode = false;
                lastError = targetTPS - vel;
                iSum = 0.0; // keep as in your opmode
            }
        } else {
            if (frac <= SPINUP_EXIT_FRAC) {
                spinupMode = true;
                iSum = 0.0;
                // readiness resets when we droop hard
                readySinceMs = 0;
                readyStable = false;
            }
        }

        double cmdPower;

        if (spinupMode) {
            cmdPower = 1.0;
            lastFF = 0.0;
            lastPTerm = 0.0;
            lastITerm = 0.0;
            lastDTerm = 0.0;
        } else {
            // PIDF hold
            double error = targetTPS - vel;
            double dError = (error - lastError) / dt;
            lastError = error;

            double ff = kF * targetTPS;

            // Anti-windup: only integrate when near target and not saturated
            double nearFrac = (targetTPS <= 1.0) ? 0.0 : (Math.abs(error) / targetTPS);
            boolean nearTarget = nearFrac <= 0.10; // within 10%

            double unclipped = ff + (kP * error) + (kD * dError) + (kI * iSum);

            boolean wouldSaturateHigh = unclipped >= (MAX_POWER - 0.02);
            boolean wouldSaturateLow  = unclipped <= (MIN_POWER + 0.02);

            if (nearTarget && !wouldSaturateHigh && !wouldSaturateLow) {
                iSum += error * dt;
                iSum = Range.clip(iSum, -I_CLAMP, I_CLAMP);
            }

            double pTerm = kP * error;
            double dTerm = kD * dError;
            double iTerm = kI * iSum;

            cmdPower = ff + pTerm + dTerm + iTerm;
            cmdPower = Range.clip(cmdPower, MIN_POWER, MAX_POWER);

            lastFF = ff;
            lastPTerm = pTerm;
            lastDTerm = dTerm;
            lastITerm = iTerm;
        }

        setMotorPower(cmdPower);
        lastCmdPower = cmdPower;

        // Ready gating: within tolerance, held for READY_HOLD_MS, only in HOLD mode
        long nowMs = System.currentTimeMillis();
        boolean readyNow = (errFrac <= READY_TOLERANCE_FRAC) && !spinupMode && (targetTPS > 1.0);

        if (readyNow) {
            if (readySinceMs == 0) readySinceMs = nowMs;
        } else {
            readySinceMs = 0;
            readyStable = false;
        }

        readyStable = (readySinceMs != 0) && ((nowMs - readySinceMs) >= READY_HOLD_MS);
    }

    /** Stop motors and disable control. */
    public void stop() {
        enabled = false;
        setMotorPower(0.0);
        targetTPS = 0.0;
        resetControllerState();
    }

    // ----------------------------
    // Optional tuning / telemetry accessors
    // ----------------------------

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public boolean isSpinupMode() { return spinupMode; }

    public double getLastCmdPower() { return lastCmdPower; }
    public double getLastFF() { return lastFF; }
    public double getLastPTerm() { return lastPTerm; }
    public double getLastITerm() { return lastITerm; }
    public double getLastDTerm() { return lastDTerm; }

    public double getLastVelocityTPS() { return lastVelTPS; }
    public double getLastVelocityRPM() { return lastVelTPS * 60.0 / TICKS_PER_REV; }
    public double getLastAbsErrPct() { return lastAbsErrPct; }

    // ----------------------------
    // Internals
    // ----------------------------

    private void setMotorPower(double power) {
        power = Range.clip(power, MIN_POWER, MAX_POWER);
        if (shootMotor1 != null) shootMotor1.setPower(power);
        if (shootMotor2 != null) shootMotor2.setPower(power);
    }

    private void resetControllerState() {
        spinupMode = true;
        lastError = 0.0;
        iSum = 0.0;
        lastTimeNs = 0;

        readySinceMs = 0;
        readyStable = false;

        lastCmdPower = 0.0;
        lastFF = 0.0;
        lastPTerm = 0.0;
        lastITerm = 0.0;
        lastDTerm = 0.0;
        lastVelTPS = 0.0;
        lastAbsErrPct = 0.0;
    }
}