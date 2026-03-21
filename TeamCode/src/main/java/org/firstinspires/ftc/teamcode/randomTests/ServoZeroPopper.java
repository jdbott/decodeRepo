package org.firstinspires.ftc.teamcode.randomTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@Disabled

@TeleOp(name = "ServoTest_P1_P2", group = "Test")
public class ServoZeroPopper extends LinearOpMode {

    private DcMotorEx shootMotor1; // encoder motor (velocity source)
    private DcMotorEx shootMotor2; // mechanically linked

    private static final double TICKS_PER_REV = 28.0;

    // Target velocity in ticks/sec
    private static final double TARGET_TICKS_PER_SEC = 1250.0;

    // Ready-to-shoot tolerance (1%) and hold time
    private static final double READY_TOLERANCE_FRAC = 0.01;
    private static final long READY_HOLD_MS = 80;

    // --- Fast spin-up behavior ---
    // Full power until within this band of target.
    // Example: 0.92 => stay at 1.0 power until you hit 92% of target.
    private static final double SPINUP_ENTER_FRAC = 0.92;

    // Once you enter closed-loop, if you drop below this, go back to full power.
    // Slightly lower than enter to avoid mode-chatter.
    private static final double SPINUP_EXIT_FRAC = 0.88;

    // --- PIDF hold gains ---
    // kF is intentionally "rough." PID (especially I) will trim the rest.
    private static final double kF = 0.00058;

    // Reduced kP vs before (less aggressive near target)
    private static final double kP = 0.00030;
    private static final double kD = 0.00008;

    // Integral term to eliminate steady-state error
    private static final double kI = 0.00012;

    // Anti-windup clamp on the integral accumulator (units: tick-sec)
    private static final double I_CLAMP = 100;

    private static final double MIN_POWER = 0.0;
    private static final double MAX_POWER = 1.0;

    private static final int RUMBLE_MS = 250;

    // -------------------------------
    // Shot detection (SIGNED error + baseline + directional + confirm)
    // -------------------------------
    //
    // Goal:
    // - While the flywheel is "idling" near target, signedErrPct will hover near 0.
    // - A shot causes the flywheel to slow, so signedErrPct jumps POSITIVE quickly.
    // - Ignore recovery/overspeed (signedErrPct <= 0) and ignore noise (confirmation loops).
    //
    // Arm when |signedErrPct| is small (idling band).
    private static final double ARM_ABS_ERR_PCT = 2.0;

    // Require a jump above the idle baseline by at least this many %-points
    // (This is your "1% -> 8%" concept; baseline ~1, dipFromIdle ~7.)
    private static final double DIP_FROM_IDLE_PCT = 4.0;

    // Require that signed error is increasing by at least this many %-points since last sample
    private static final double MIN_DELTA_PCT = 2.0;

    // Must have been idling within this time window to be "armed"
    private static final long ARM_RECENT_MS = 250;

    // Baseline smoothing (LPF): 0.10–0.25 is typical; higher follows faster but is noisier
    private static final double IDLE_LPF_ALPHA = 0.15;

    // Require this many consecutive loops to confirm a real shot dip
    private static final int CONFIRM_LOOPS = 2;

    // Spike must occur within this window relative to the prior sample
    private static final long SHOT_SPIKE_WINDOW_MS = 250;

    // Rumble duration for shot detection
    private static final int SHOT_RUMBLE_MS = 200;

    // Debounce so one shot doesn't rumble multiple times
    private static final long SHOT_DEBOUNCE_MS = 450;

    @Override
    public void runOpMode() {

        shootMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shootMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        // Match your existing direction setup (adjust if needed)
        shootMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Custom loop; read velocity and command power
        shootMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Shooter velocity-hold test ready.");
        telemetry.addData("Target (t/s)", "%.0f", TARGET_TICKS_PER_SEC);
        telemetry.addData("Target RPM (approx)", "%.0f", (TARGET_TICKS_PER_SEC / TICKS_PER_REV) * 60.0);
        telemetry.addLine("Press A to toggle shooter ON/OFF.");
        telemetry.update();

        waitForStart();

        boolean shooterOn = false;
        boolean lastA = false;

        // Control state
        double lastError = 0.0;
        double iSum = 0.0;
        long lastTimeNs = System.nanoTime();

        // Ready gating state
        long readySinceMs = 0;
        boolean hasRumbled = false;

        // Mode state: are we in full-power spinup?
        boolean spinupMode = true;

        // -------------------------------
        // Shot detection state (SIGNED error path)
        // -------------------------------
        double lastSignedErrPct = 0.0;
        long lastSignedErrMs = 0;

        double idleErrPct = 0.0;          // low-pass baseline
        boolean idleBaselineValid = false;

        long lastIdleMs = 0;              // last time we were in idling band
        int confirmCount = 0;             // consecutive candidate confirmations

        long lastShotDetectMs = 0;        // debounce
        boolean shotSpikeDetected = false;

        while (opModeIsActive()) {

            // Toggle shooter with A
            boolean a = gamepad1.a;
            if (a && !lastA) {
                shooterOn = !shooterOn;

                // Reset controller state on toggle
                readySinceMs = 0;
                hasRumbled = false;
                lastError = 0.0;
                iSum = 0.0;
                lastTimeNs = System.nanoTime();
                spinupMode = true;

                // Reset shot detection state on toggle
                lastSignedErrPct = 0.0;
                lastSignedErrMs = 0;
                idleErrPct = 0.0;
                idleBaselineValid = false;
                lastIdleMs = 0;
                confirmCount = 0;
                lastShotDetectMs = 0;
                shotSpikeDetected = false;
            }
            lastA = a;

            // NOTE: You said you're reading the correct velocity already; leaving your original line.
            double vel = shootMotor2.getVelocity(); // ticks/sec (encoder motor)

            // Unsigned percent error for general telemetry + ready logic
            double errorNow = TARGET_TICKS_PER_SEC - vel;
            double errFrac = (TARGET_TICKS_PER_SEC <= 1.0) ? 0.0 : (Math.abs(errorNow) / TARGET_TICKS_PER_SEC);
            double errPct = errFrac * 100.0;

            // Signed percent error for shot detection:
            //  + => below target (slower)
            //  - => above target (faster)
            double signedErrPct = (TARGET_TICKS_PER_SEC <= 1.0)
                    ? 0.0
                    : ((TARGET_TICKS_PER_SEC - vel) / TARGET_TICKS_PER_SEC) * 100.0;

            long nowMs = System.currentTimeMillis();
            shotSpikeDetected = false;

            if (shooterOn) {
                // Timing
                long nowNs = System.nanoTime();
                double dt = (nowNs - lastTimeNs) / 1e9;
                if (dt <= 0) dt = 1e-3;
                lastTimeNs = nowNs;

                // Decide spinup vs closed-loop hold
                double frac = (TARGET_TICKS_PER_SEC <= 1.0) ? 0.0 : (vel / TARGET_TICKS_PER_SEC);

                if (spinupMode) {
                    // Stay full power until we are close enough
                    if (frac >= SPINUP_ENTER_FRAC) {
                        spinupMode = false;
                        // When entering closed-loop, reset derivative memory
                        lastError = TARGET_TICKS_PER_SEC - vel;
                        // (keep iSum = 0 to avoid overshoot after bang-bang)
                        iSum = 0.0;

                        // Also (re)seed the idle baseline once we enter HOLD
                        idleBaselineValid = false;
                        lastIdleMs = 0;
                        confirmCount = 0;
                    }
                } else {
                    // If we droop hard (e.g., shot load), revert to full power
                    if (frac <= SPINUP_EXIT_FRAC) {
                        spinupMode = true;
                        iSum = 0.0; // prevent windup when pinned

                        // While in spinup, we do not want shot detection to fire.
                        // Keep baseline, but clear confirmations.
                        confirmCount = 0;
                    }
                }

                double cmdPower;

                if (spinupMode) {
                    // Full power spin-up
                    cmdPower = 1.0;

                    shootMotor1.setPower(cmdPower);
                    shootMotor2.setPower(cmdPower);

                } else {
                    // PIDF hold
                    double error = TARGET_TICKS_PER_SEC - vel;
                    double dError = (error - lastError) / dt;
                    lastError = error;

                    double ff = kF * TARGET_TICKS_PER_SEC;

                    // Anti-windup: only integrate when near target and not saturated
                    double nearFrac = Math.abs(error) / TARGET_TICKS_PER_SEC;
                    boolean nearTarget = nearFrac <= 0.10; // within 10%

                    double unclipped = ff + (kP * error) + (kD * dError) + (kI * iSum);

                    boolean wouldSaturateHigh = unclipped >= (MAX_POWER - 0.02);
                    boolean wouldSaturateLow  = unclipped <= (MIN_POWER + 0.02);

                    if (nearTarget && !wouldSaturateHigh && !wouldSaturateLow) {
                        iSum += error * dt;
                        iSum = Range.clip(iSum, -I_CLAMP, I_CLAMP);
                    }

                    cmdPower = ff + (kP * error) + (kD * dError) + (kI * iSum);
                    cmdPower = Range.clip(cmdPower, MIN_POWER, MAX_POWER);

                    shootMotor1.setPower(cmdPower);
                    shootMotor2.setPower(cmdPower);
                }

                // -------------------------------
                // Shot detection (SIGNED error + baseline + directional + confirm)
                // Must run AFTER spinupMode is finalized for this loop.
                // -------------------------------
                if (lastSignedErrMs == 0) lastSignedErrMs = nowMs;

                long dMs = nowMs - lastSignedErrMs;
                double dSignedErrPct = signedErrPct - lastSignedErrPct;

                // Idling band: close to target (either slightly above or below)
                boolean idlingNow = !spinupMode && (Math.abs(signedErrPct) <= ARM_ABS_ERR_PCT);

                // While idling, update arm timer + maintain baseline (LPF)
                if (idlingNow) {
                    lastIdleMs = nowMs;

                    if (!idleBaselineValid) {
                        idleErrPct = signedErrPct;
                        idleBaselineValid = true;
                    } else {
                        idleErrPct = (1.0 - IDLE_LPF_ALPHA) * idleErrPct + IDLE_LPF_ALPHA * signedErrPct;
                    }
                }

                boolean armedRecently = idleBaselineValid
                        && (lastIdleMs != 0)
                        && ((nowMs - lastIdleMs) <= ARM_RECENT_MS);

                // Directional filter: only detect when we are slowing down (below target AND error increasing)
                boolean dipDirectionOK = (signedErrPct > 0.0) && (dSignedErrPct > 0.0);

                // Magnitude above baseline (your "idle ~1 then jump to ~8")
                double dipFromIdle = signedErrPct - idleErrPct;

                boolean dipCandidate =
                        !spinupMode &&
                                armedRecently &&
                                dipDirectionOK &&
                                (dMs > 0 && dMs <= SHOT_SPIKE_WINDOW_MS) &&
                                (dSignedErrPct >= MIN_DELTA_PCT) &&
                                (dipFromIdle >= DIP_FROM_IDLE_PCT) &&
                                (nowMs - lastShotDetectMs >= SHOT_DEBOUNCE_MS);

                // Confirm across multiple loops to reject one-frame noise
                if (dipCandidate) confirmCount++;
                else confirmCount = 0;

                shotSpikeDetected = (confirmCount >= CONFIRM_LOOPS);

                if (shotSpikeDetected) {
                    gamepad1.rumble(1.0, 0.3, SHOT_RUMBLE_MS);
                    lastShotDetectMs = nowMs;
                    confirmCount = 0;

                    // Optional: reset ready rumble state so you can re-rumble when stabilized again
                    readySinceMs = 0;
                    hasRumbled = false;
                }

                // Update history for next loop
                lastSignedErrPct = signedErrPct;
                lastSignedErrMs = nowMs;

                // -------------------------------
                // Ready check (within 1% for READY_HOLD_MS)
                // -------------------------------
                boolean readyNow = errFrac <= READY_TOLERANCE_FRAC && !spinupMode;

                long ms = nowMs;
                if (readyNow) {
                    if (readySinceMs == 0) readySinceMs = ms;
                } else {
                    readySinceMs = 0;
                    hasRumbled = false;
                }

                boolean readyStable = (readySinceMs != 0) && ((ms - readySinceMs) >= READY_HOLD_MS);

                if (readyStable && !hasRumbled) {
                    //gamepad1.rumble(1.0, 0.3, RUMBLE_MS);
                    hasRumbled = true;
                }

                // -------------------------------
                // Telemetry
                // -------------------------------
                telemetry.addData("Shooter", "ON");
                telemetry.addData("Mode", spinupMode ? "SPINUP (1.0 power)" : "HOLD (PIDF)");
                telemetry.addData("Target (t/s)", "%.0f", TARGET_TICKS_PER_SEC);
                telemetry.addData("Actual (t/s)", "%.0f", vel);
                telemetry.addData("Target RPM", "%.0f", (TARGET_TICKS_PER_SEC / TICKS_PER_REV) * 60.0);
                telemetry.addData("Actual RPM", "%.0f", (vel / TICKS_PER_REV) * 60.0);

                telemetry.addData("Abs Error (%)", "%.2f", errPct);
                telemetry.addData("SignedErr(%)", "%.2f", signedErrPct);

                telemetry.addData("Cmd Power", "%.3f", shootMotor1.getPower());
                telemetry.addData("iSum", "%.3f", iSum);
                telemetry.addData("Ready", readyStable);

                // Shot detection debug
                telemetry.addData("ShotDipDetected", shotSpikeDetected);
                telemetry.addData("IdleBaselineValid", idleBaselineValid);
                telemetry.addData("IdleErr(%)", "%.2f", idleErrPct);
                telemetry.addData("DipFromIdle(%)", "%.2f", (signedErrPct - idleErrPct));
                telemetry.addData("dSignedErr(%)", "%.2f", dSignedErrPct);
                telemetry.addData("ConfirmCount", confirmCount);
                telemetry.addData("ArmedRecently", armedRecently);

                double radPerSec =
                        (vel / TICKS_PER_REV) * (2.0 * Math.PI);

                telemetry.addData("LOOK AT ME (rad/s)", "%.2f", radPerSec);

            } else {
                shootMotor1.setPower(0);
                shootMotor2.setPower(0);

                readySinceMs = 0;
                hasRumbled = false;
                lastError = 0.0;
                iSum = 0.0;
                lastTimeNs = System.nanoTime();
                spinupMode = true;

                // Reset shot detection state while OFF
                lastSignedErrPct = 0.0;
                lastSignedErrMs = 0;
                idleErrPct = 0.0;
                idleBaselineValid = false;
                lastIdleMs = 0;
                confirmCount = 0;
                lastShotDetectMs = 0;
                shotSpikeDetected = false;

                telemetry.addData("Shooter", "OFF");
                telemetry.addData("Actual (t/s)", "%.0f", vel);
                telemetry.addData("Actual RPM", "%.0f", (vel / TICKS_PER_REV) * 60.0);
                telemetry.addData("ShotDipDetected", false);
            }

            telemetry.update();
        }
    }
}
