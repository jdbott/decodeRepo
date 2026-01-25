package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter velocity controller with:
 *  - Full-power spinup until near target
 *  - Fast recovery ("boost") after a droop event
 *  - PIDF hold once near target
 *  - "Ready" gating (within tolerance for hold time)
 *
 * Notes on changes vs your prior version:
 *  1) More aggressive re-entry to full-power after droop (SPINUP_EXIT_FRAC raised)
 *  2) Added a short "boost window" after a droop to snap back faster
 *  3) Improved integral behavior:
 *     - Do NOT reset integral on spinup->hold transitions
 *     - Integrate even while saturated (with back-calculation anti-windup) so recovery isn't sluggish
 *     - Optional slow decay of integral when disabled / target=0
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
    // Enter HOLD when we are close enough to target
    private static final double SPINUP_ENTER_FRAC = 0.88;

    // Re-enter SPINUP more aggressively when we droop.
    // Raised from 0.91 so even moderate droop kicks you back to 1.0 power.
    private static final double SPINUP_EXIT_FRAC  = 0.9;

    // --- Droop boost behavior ---
    // If droop exceeds this fraction of target, apply full power for BOOST_MS.
    // This triggers even if you haven't crossed SPINUP_EXIT_FRAC yet.
    private static final double BOOST_DROOP_FRAC = 0.04;   // 4% droop
    private static final long   BOOST_MS         = 0;    // 80-200ms typical

    // --- PIDF hold gains ---
    private double kF = 0.00058;
    private double kP = 0.00030;
    private double kD = 0.00008;
    private double kI = 0.00012;

    // Anti-windup clamp on integral accumulator (tick-sec)
    private static final double I_CLAMP = 200.0; // increased a bit for stronger recovery authority

    // Back-calculation strength for anti-windup (higher = unwind faster when saturated)
    // Units roughly: 1/sec. Keep modest to avoid oscillation.
    private static final double AW_BACKCALC = 6.0;

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

    // Droop boost state
    private long boostUntilMs = 0;

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
    // Public API
    // ----------------------------

    /** Enable/disable shooter control (like shooterOn toggle). */
    public void setEnabled(boolean enabled) {
        if (this.enabled != enabled) {
            this.enabled = enabled;

            // Keep integral rather than nuking it; just reset timing/ready/boost.
            lastTimeNs = 0;
            lastError = 0.0;
            readySinceMs = 0;
            readyStable = false;
            boostUntilMs = 0;

            if (!enabled) {
                setMotorPower(0.0);
            } else {
                // Re-enter spinup when first enabling (but keep iSum)
                spinupMode = true;
            }
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Set desired velocity in ticks/sec. */
    public void setTargetTPS(double ticksPerSec) {
        targetTPS = Math.max(0.0, ticksPerSec);

        // If target is effectively zero, don't keep integral piled up.
        if (targetTPS <= 1.0) {
            iSum *= 0.5; // mild decay, avoids weirdness next time you spin up
        }

        if (enabled) {
            // Typically re-enter spinup on target changes, but keep iSum (don’t reset).
            spinupMode = true;
            readySinceMs = 0;
            readyStable = false;
            boostUntilMs = 0;
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
        //if (shootMotor2 == null) return 0.0;
        return -shootMotor1.getVelocity();
    }

    public double getVelocityRPM() {
        return getVelocityTPS() * 60.0 / TICKS_PER_REV;
    }

    /** True when within tolerance for READY_HOLD_MS and not in spinup/boost mode. */
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

        long nowMs = System.currentTimeMillis();

        // Fraction of target achieved
        double frac = (targetTPS <= 1.0) ? 0.0 : (vel / targetTPS);

        // ----------------------------
        // Droop detection -> boost window
        // ----------------------------
        // If we are in HOLD (not spinup) and we droop by more than BOOST_DROOP_FRAC,
        // slam to full power briefly to refill flywheel energy quickly.
        if (!spinupMode && targetTPS > 1.0) {
            double droopFrac = 1.0 - frac; // positive when below target
            if (droopFrac >= BOOST_DROOP_FRAC) {
                boostUntilMs = Math.max(boostUntilMs, nowMs + BOOST_MS);
            }
        }

        boolean boostActive = (nowMs < boostUntilMs);

        // ----------------------------
        // Decide spinup vs hold
        // ----------------------------
        if (spinupMode) {
            if (frac >= SPINUP_ENTER_FRAC) {
                spinupMode = false;
                // Do NOT reset integral; only refresh derivative baseline.
                lastError = targetTPS - vel;
            }
        } else {
            // More aggressive re-entry to spinup when drooping
            if (frac <= SPINUP_EXIT_FRAC) {
                spinupMode = true;
                // readiness resets when we droop
                readySinceMs = 0;
                readyStable = false;
                // keep iSum
            }
        }

        // ----------------------------
        // Compute command
        // ----------------------------
        double cmdPower;

        if (spinupMode || boostActive) {
            // Full power in spinup or boost.
            cmdPower = 1.0;

            lastFF = 0.0;
            lastPTerm = 0.0;
            lastITerm = 0.0;
            lastDTerm = 0.0;

            // While saturating high, unwind integral via back-calculation to prevent slow recovery/overshoot.
            // This avoids "stuck integral" when you're pinned at 1.0 for a while.
            if (targetTPS > 1.0) {
                double error = targetTPS - vel;
                // Desired unclipped = 1.0, so actual - unclipped approx 0; but we still want to prevent runaway iSum.
                // Use a gentle decay under full-power to keep iSum sane.
                iSum *= Math.max(0.0, 1.0 - (0.8 * dt));
                iSum = Range.clip(iSum, -I_CLAMP, I_CLAMP);
                lastError = error;
            }
        } else {
            // PIDF hold
            double error = targetTPS - vel;
            double dError = (error - lastError) / dt;
            lastError = error;

            double ff = kF * targetTPS;

            double pTerm = kP * error;
            double dTerm = kD * dError;
            double iTerm = kI * iSum;

            double unclipped = ff + pTerm + dTerm + iTerm;
            double clipped = Range.clip(unclipped, MIN_POWER, MAX_POWER);

            // Back-calculation anti-windup:
            // If we clip, feed back the difference to iSum so it doesn't wind up and doesn't get "slow" after saturation.
            // Integrate error always, but correct for saturation.
            double satError = (clipped - unclipped); // negative when we're clipped high, positive when clipped low
            iSum += (error * dt) + (AW_BACKCALC * satError * dt / Math.max(1e-6, kI)); // scale by kI so effect is consistent
            iSum = Range.clip(iSum, -I_CLAMP, I_CLAMP);

            // Recompute with updated integral (optional but helps)
            iTerm = kI * iSum;
            unclipped = ff + pTerm + dTerm + iTerm;
            clipped = Range.clip(unclipped, MIN_POWER, MAX_POWER);

            cmdPower = clipped;

            lastFF = ff;
            lastPTerm = pTerm;
            lastDTerm = dTerm;
            lastITerm = iTerm;
        }

        setMotorPower(cmdPower);
        lastCmdPower = cmdPower;

        // ----------------------------
        // Ready gating
        // ----------------------------
        // Only ready when not spinup and not boosting.
        boolean readyNow = (errFrac <= READY_TOLERANCE_FRAC)
                && !spinupMode
                && !boostActive
                && (targetTPS > 1.0);

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

        // Keep iSum from being weird next time; don’t hard reset unless you want to.
        iSum = 0.0;

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
        lastTimeNs = 0;

        boostUntilMs = 0;

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