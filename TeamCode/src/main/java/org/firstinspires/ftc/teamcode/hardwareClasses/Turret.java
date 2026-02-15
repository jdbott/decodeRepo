package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Turret {
    private DcMotorEx turretMotor;

    // Constants
    private final double ticksPerRev = 145.1;
    private final double gearRatio = (108.0 / 18.0) * 2.0; // 12:1 motor revs per turret rev
    private final double ticksPerDegree = (ticksPerRev * gearRatio) / 360.0;

    // Tunable parameters
    private double kP = 0.01;
    private double kF = 0.003; // feedforward gain (adjust in test)
    private double minPower = 0.05;
    private double maxAngle = 160;
    private double minAngle = -160;

    // State variables
    private double lastSetAngle = 0;  // degrees
    private boolean turretBusy = false;
    private double feedforward = 0;

    // ===== Overcurrent / hard-stop protection =====
    // Tune these:
    private double CURRENT_LIMIT_AMPS = 6.0;          // <-- tune this
    private long   OVERCURRENT_DEBOUNCE_MS = 120;     // must be high for this long to trigger
    private long   RECOVERY_COOLDOWN_MS = 400;        // ignore current trip for a bit after triggering
    private double BACKOFF_DEG = 45.0;                // how far to back away

    private long overCurrentStartMs = -1;
    private long recoveryIgnoreUntilMs = 0;
    private boolean recovering = false;

    // Initialization
    public void init(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turretMotor.setDirection(direction);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // Add this back:
    public void setLimits(double minDeg, double maxDeg) {
        minAngle = minDeg;
        maxAngle = maxDeg;
    }

    // ===== Public Configuration =====
    public void setKP(double newKP) { kP = newKP; }
    public void setKF(double newKF) { kF = newKF; }
    public void setMinPower(double minPwr) { minPower = Math.abs(minPwr); }

    // Overcurrent tuning
    public void setCurrentLimitAmps(double amps) { CURRENT_LIMIT_AMPS = Math.max(0, amps); }
    public void setOvercurrentDebounceMs(long ms) { OVERCURRENT_DEBOUNCE_MS = Math.max(0, ms); }
    public void setRecoveryCooldownMs(long ms) { RECOVERY_COOLDOWN_MS = Math.max(0, ms); }
    public void setBackoffDeg(double deg) { BACKOFF_DEG = Math.abs(deg); }

    // Feedforward setter
    public void setFeedforward(double angularVelDegPerSec) {
        feedforward = angularVelDegPerSec * kF;
        feedforward = Range.clip(feedforward, -1.0, 1.0);
    }

    // ===== Motion Control =====
    public void setAngle(double targetDeg) {
        targetDeg = Range.clip(targetDeg, minAngle, maxAngle);
        lastSetAngle = targetDeg;
        turretBusy = true;
        recovering = false;
        overCurrentStartMs = -1;
    }

    public void update() {
        // Safety: check for overcurrent *only while we’re actually trying to move*
        // (If you want it always active, you can remove the turretBusy check.)
        if (turretBusy) {
            if (checkAndHandleOvercurrent()) {
                // we triggered a recovery; the handler already changed target/flags
                return;
            }
            moveTurretToAngle(lastSetAngle);
        } else {
            // Optional: don’t “hold position” during recovery cooldown if you want it to relax.
            // Keeping your current behavior (hold) but still allowing feedforward.
            correctTurretPosition();
        }
    }

    private boolean checkAndHandleOvercurrent() {
        long now = System.currentTimeMillis();

        // Ignore trips briefly right after we trigger recovery (prevents immediate re-trigger).
        if (now < recoveryIgnoreUntilMs) return false;

        // If we are not commanding meaningful motion, don’t trip.
        // (Prevents false trips when holding near target.)
        double errorDeg = lastSetAngle - getCurrentAngle();
        if (Math.abs(errorDeg) < 2.0) { // close enough; don’t treat current as a jam
            overCurrentStartMs = -1;
            return false;
        }

        double amps = getMotorCurrentAmps();

        if (amps >= CURRENT_LIMIT_AMPS && CURRENT_LIMIT_AMPS > 0) {
            if (overCurrentStartMs < 0) overCurrentStartMs = now;

            if (now - overCurrentStartMs >= OVERCURRENT_DEBOUNCE_MS) {
                // Jam / hard-stop detected: stop + back off 45° opposite the direction we were trying to go.
                turretMotor.setPower(0);

                double dir = Math.signum(errorDeg); // + if we were trying to increase angle, - otherwise
                if (dir == 0) dir = 1; // fallback

                double backoffTarget = getCurrentAngle() - dir * BACKOFF_DEG;
                backoffTarget = Range.clip(backoffTarget, minAngle, maxAngle);

                lastSetAngle = backoffTarget;
                turretBusy = true;       // keep moving to the new target
                recovering = true;

                // Reset timers + set cooldown
                overCurrentStartMs = -1;
                recoveryIgnoreUntilMs = now + RECOVERY_COOLDOWN_MS;

                return true;
            }
        } else {
            // current dropped back below threshold, reset debounce
            overCurrentStartMs = -1;
        }

        return false;
    }

    private double getMotorCurrentAmps() {
        try {
            return turretMotor.getCurrent(CurrentUnit.AMPS);
        } catch (Exception e) {
            // If current is unsupported, you can’t do this reliably.
            // Return 0 so we never trip.
            return 0.0;
        }
    }

    private void moveTurretToAngle(double targetAngle) {
        double targetTicks = targetAngle * ticksPerDegree;
        double currentTicks = turretMotor.getCurrentPosition();
        double error = targetTicks - currentTicks;

        if (Math.abs(error) > (0.25 * ticksPerDegree)) { // ~0.25° tolerance
            double power = kP * error + feedforward;
            power = applyMinPower(power);
            turretMotor.setPower(Range.clip(power, -1, 1));
        } else {
            turretMotor.setPower(0);
            turretBusy = false;
            recovering = false;
        }
    }

    private void correctTurretPosition() {
        double targetTicks = lastSetAngle * ticksPerDegree;
        double currentTicks = turretMotor.getCurrentPosition();
        double error = targetTicks - currentTicks;
        double power = kP * error + feedforward;
        power = applyMinPower(power);
        turretMotor.setPower(Range.clip(power, -1, 1));
    }

    private double applyMinPower(double power) {
        if (minPower == 0) return power;
        if (power > 0 && power < minPower) return minPower;
        if (power < 0 && power > -minPower) return -minPower;
        return power;
    }

    // ===== Accessors =====
    public double getCurrentAngle() { return turretMotor.getCurrentPosition() / ticksPerDegree; }
    public double getTargetAngle() { return lastSetAngle; }
    public double getError() { return lastSetAngle - getCurrentAngle(); }
    public boolean isBusy() { return turretBusy; }
    public boolean isRecovering() { return recovering; }
    public double getMotorCurrent() { return getMotorCurrentAmps(); }

    public void stop() {
        turretMotor.setPower(0);
        turretBusy = false;
        recovering = false;
        overCurrentStartMs = -1;
    }

    public void zeroTurret() {
        turretMotor.setPower(0);
        turretBusy = false;
        recovering = false;
        feedforward = 0;

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastSetAngle = 0;

        overCurrentStartMs = -1;
        recoveryIgnoreUntilMs = 0;
    }
}