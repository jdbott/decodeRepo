package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/**
 * Robust self-tuning linear slides with trapezoidal motion profiling.
 */
public class offLinearSlides {

    private final DcMotorEx slideMotor;

    /* ================= HARDWARE CONSTANTS ================= */
    private static final double TICKS_PER_REV = 145.1;
    private static final double SPOOL_DIAMETER_IN = 1.504;
    private static final double INCHES_PER_REV = Math.PI * SPOOL_DIAMETER_IN;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;

    /* ================= LIMITS & PRESETS ================= */
    private double maxExtensionInches = 24.0;
    private double minExtensionInches = 0.0;
    private double maxPower = 1.0;

    public static final double RETRACTED = 0.0;
    public static final double LOW = 8.5;
    public static final double MID = 17.0;
    public static final double HIGH = 26.0;

    /* ================= PID & FEEDFORWARD ================= */
    // TUNED VALUES — paste your own here after running the tuner
    private double kP = 0.08;
    private double kI = 0.00000;
    private double kD = 0.004;
    private double kV = 0.0366;
    private double kA = 0.008;
    private double kS = 0.15;  // STATIC FRICTION — critical for startup!
    private double kG = 0.0;   // gravity (vertical slides only)

    private double integralSum = 0.0;
    private double lastPosition = 0.0;
    private double lastTime = 0.0;

    /* ================= MOTION PROFILE ================= */
    private double profileMaxVel = 35.0;
    private double profileMaxAccel = 80.0;
    private double profileStartTime = 0.0;
    private double profileStartPos = 0.0;
    private double profileTargetPos = 0.0;
    private boolean profileActive = false;

    /* ================= AUTO-TUNER ================= */
    public enum TuneState { IDLE, CALIBRATE_FF, RELAY, CALCULATE, VALIDATE, DONE }
    private TuneState tuneState = TuneState.IDLE;

    private final ElapsedTime tuneTimer = new ElapsedTime();
    private final ArrayList<Double> tunePeriods = new ArrayList<>();
    private double tuneMaxPos = -Double.MAX_VALUE;
    private double tuneMinPos = Double.MAX_VALUE;
    private double tuneLastCrossingTime = 0.0;
    private double tuneLastError = 0.0;
    private int tuneCycleCount = 0;
    private double tuneTargetPos = 0.0;
    private double tuneValidatedOvershoot = 0.0;

    private static final double TUNE_POWER = 0.7;
    private static final int REQUIRED_CYCLES = 4;
    private static final double TUNE_DEADBAND = 0.15;

    /* ================= RUNTIME ================= */
    private final ElapsedTime runtime = new ElapsedTime();

    /* ================= CONSTRUCTORS ================= */
    public offLinearSlides(HardwareMap hardwareMap) {
        this(hardwareMap, "cool", false);
    }

    public offLinearSlides(HardwareMap hardwareMap, String motorName, boolean reversed) {
        slideMotor = hardwareMap.get(DcMotorEx.class, motorName);
        slideMotor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ================= CONFIGURATION ================= */
    public void setMaxPower(double power) {
        maxPower = Range.clip(Math.abs(power), 0.0, 1.0);
    }

    public void setLimits(double minInches, double maxInches) {
        minExtensionInches = minInches;
        maxExtensionInches = maxInches;
    }

    public void setGravityFeedforward(double kg) {
        kG = kg;
    }

    public void setProfileConstraints(double maxVelInPerSec, double maxAccelInPerSecSq) {
        profileMaxVel = maxVelInPerSec;
        profileMaxAccel = maxAccelInPerSecSq;
    }

    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public void setkA(double ka) {
        this.kA = ka;
    }

    public void setkS(double ks) {
        this.kS = ks;
    }

    /** Flip motor direction if slides move the wrong way. Call before waitForStart(). */
    public void reverseMotorDirection() {
        if (slideMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        // Reset encoder to keep sign consistent
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ================= MANUAL CONTROL ================= */
    public void setPower(double power) {
        cancelMotion();
        double current = getCurrentPositionInches();
        if ((current >= maxExtensionInches && power > 0) ||
                (current <= minExtensionInches && power < 0)) {
            slideMotor.setPower(0);
            return;
        }
        slideMotor.setPower(Range.clip(power, -maxPower, maxPower));
    }

    public void up()   { setPower(maxPower); }
    public void down() { setPower(-maxPower); }

    /* ================= POSITION CONTROL ================= */
    public void goToPreset(double presetInches) {
        setTargetPosition(presetInches);
    }

    public void setTargetPosition(double inches) {
        inches = Range.clip(inches, minExtensionInches, maxExtensionInches);
        profileStartPos = getCurrentPositionInches();
        profileTargetPos = inches;
        profileStartTime = runtime.seconds();
        profileActive = true;
        integralSum = 0.0;
        lastPosition = profileStartPos;
        lastTime = runtime.seconds();
    }

    public void cancelMotion() {
        profileActive = false;
        if (tuneState != TuneState.IDLE && tuneState != TuneState.DONE) {
            tuneState = TuneState.IDLE;
        }
        slideMotor.setPower(0);
    }

    /* ================= MAIN UPDATE LOOP ================= */
    public void update() {
        double now = runtime.seconds();
        double dt = now - lastTime;
        if (dt <= 0 || dt > 0.5) dt = 0.001;
        lastTime = now;

        if (tuneState != TuneState.IDLE) {
            updateTuner(dt);
            return;
        }

        if (!profileActive) return;

        // 1. Motion profile setpoint
        double[] setpoint = calculateTrapezoidalProfile(
                profileStartPos, profileTargetPos,
                profileMaxVel, profileMaxAccel,
                now - profileStartTime
        );
        double targetPos = setpoint[0];
        double targetVel = setpoint[1];
        double targetAccel = setpoint[2];

        // 2. Sensor readings (manual velocity = derivative of position)
        double currentPos = getCurrentPositionInches();
        double currentVel = (currentPos - lastPosition) / dt;
        lastPosition = currentPos;

        // 3. PID (derivative on measurement)
        double error = targetPos - currentPos;
        integralSum += error * dt;

        double maxIntegral = (kI > 0.001) ? (maxPower / kI) : 0.0;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);

        double pidOutput = (kP * error) + (kI * integralSum) - (kD * currentVel);

        // 4. Feedforward (CRITICAL: kS breaks static friction!)
        double staticFF = 0.0;
        if (Math.abs(targetVel) > 0.01) {
            staticFF = kS * Math.signum(targetVel);
        }
        double ffOutput = staticFF + (kV * targetVel) + (kA * targetAccel) + kG;

        double output = pidOutput + ffOutput;
        output = Range.clip(output, -maxPower, maxPower);

        slideMotor.setPower(output);

        // 5. Settle check
        if (Math.abs(error) < 0.05 && Math.abs(currentVel) < 0.1) {
            profileActive = false;
        }
    }

    /* ================= MOTION PROFILE MATH ================= */
    private double[] calculateTrapezoidalProfile(double start, double target,
                                                 double maxV, double maxA, double t) {
        double dir = Math.signum(target - start);
        double dist = Math.abs(target - start);

        double accelTime = maxV / maxA;
        double accelDist = 0.5 * maxA * accelTime * accelTime;

        double cruiseTime, totalTime;
        if (2.0 * accelDist >= dist) {
            accelTime = Math.sqrt(dist / maxA);
            accelDist = 0.5 * maxA * accelTime * accelTime;
            cruiseTime = 0.0;
            totalTime = 2.0 * accelTime;
        } else {
            cruiseTime = (dist - 2.0 * accelDist) / maxV;
            totalTime = 2.0 * accelTime + cruiseTime;
        }

        double pos, vel, accel;
        if (t < 0.0) {
            pos = start; vel = 0.0; accel = 0.0;
        } else if (t < accelTime) {
            accel = maxA * dir;
            vel = maxA * t * dir;
            pos = start + 0.5 * maxA * t * t * dir;
        } else if (t < accelTime + cruiseTime) {
            accel = 0.0;
            vel = maxV * dir;
            pos = start + (accelDist + maxV * (t - accelTime)) * dir;
        } else if (t < totalTime) {
            double td = totalTime - t;
            accel = -maxA * dir;
            vel = maxA * td * dir;
            pos = target - 0.5 * maxA * td * td * dir;
        } else {
            pos = target; vel = 0.0; accel = 0.0;
        }

        return new double[]{pos, vel, accel};
    }

    /* ================= AUTO-TUNER ================= */
    public void startAutoTune() {
        tuneState = TuneState.CALIBRATE_FF;
        tuneTimer.reset();
        tunePeriods.clear();
        tuneMaxPos = -Double.MAX_VALUE;
        tuneMinPos = Double.MAX_VALUE;
        tuneCycleCount = 0;
        tuneLastCrossingTime = 0.0;
        tuneLastError = 0.0;
        tuneValidatedOvershoot = 0.0;
        integralSum = 0.0;
        lastPosition = getCurrentPositionInches();
    }

    public boolean isTuning() {
        return tuneState != TuneState.IDLE && tuneState != TuneState.DONE;
    }

    public boolean isTuningComplete() {
        return tuneState == TuneState.DONE;
    }

    public TuneState getTuneState() {
        return tuneState;
    }

    public String getTunedGainsString() {
        return String.format("setPID(%.5f, %.5f, %.5f); // kV=%.5f kA=%.5f kS=%.5f", kP, kI, kD, kV, kA, kS);
    }

    private void updateTuner(double dt) {
        double currentPos = getCurrentPositionInches();
        double currentVel = (currentPos - lastPosition) / dt;
        lastPosition = currentPos;

        switch (tuneState) {
            case CALIBRATE_FF:
                double calPower = 0.5 * maxPower;
                slideMotor.setPower(calPower);
                if (tuneTimer.seconds() > 0.75) {
                    if (Math.abs(currentVel) > 1.0) {
                        kV = calPower / Math.abs(currentVel);
                    }
                    tuneTargetPos = (maxExtensionInches + minExtensionInches) / 2.0;
                    tuneTimer.reset();
                    tuneState = TuneState.RELAY;
                }
                break;

            case RELAY:
                double error = tuneTargetPos - currentPos;
                double relayPower;
                if (Math.abs(error) < TUNE_DEADBAND) {
                    relayPower = 0.0;
                } else {
                    relayPower = Math.signum(error) * TUNE_POWER * maxPower;
                }
                slideMotor.setPower(relayPower);

                if (currentPos > tuneMaxPos) tuneMaxPos = currentPos;
                if (currentPos < tuneMinPos) tuneMinPos = currentPos;

                if (tuneLastError != 0.0 && Math.signum(error) != Math.signum(tuneLastError)) {
                    double now = tuneTimer.seconds();
                    if (tuneLastCrossingTime > 0.0) {
                        double period = now - tuneLastCrossingTime;
                        if (period > 0.1) {
                            tunePeriods.add(period);
                            tuneCycleCount++;
                        }
                    }
                    tuneLastCrossingTime = now;
                }
                tuneLastError = error;

                if (tuneCycleCount >= REQUIRED_CYCLES) {
                    tuneState = TuneState.CALCULATE;
                } else if (tuneTimer.seconds() > 10.0) {
                    kP = 0.08; kI = 0.0; kD = 0.004;
                    tuneState = TuneState.DONE;
                }
                break;

            case CALCULATE:
                if (tunePeriods.size() >= 2) {
                    double avgPeriod = 0.0;
                    for (double p : tunePeriods) avgPeriod += p;
                    avgPeriod /= tunePeriods.size();

                    double amplitude = (tuneMaxPos - tuneMinPos) / 2.0;
                    if (amplitude < 0.1) amplitude = 0.1;

                    double Ku = (4.0 * TUNE_POWER * maxPower) / (Math.PI * amplitude);

                    kP = Ku / 3.0;
                    kI = 2.0 * kP / avgPeriod;
                    kD = kP * avgPeriod / 3.0;

                    kP = Range.clip(kP, 0.0, 2.0);
                    kI = Range.clip(kI, 0.0, 1.0);
                    kD = Range.clip(kD, 0.0, 0.5);
                } else {
                    kP = 0.08; kI = 0.0; kD = 0.004;
                }

                tuneTimer.reset();
                profileStartPos = currentPos;
                profileTargetPos = Range.clip(tuneTargetPos + 6.0, minExtensionInches, maxExtensionInches);
                integralSum = 0.0;
                lastPosition = currentPos;
                tuneState = TuneState.VALIDATE;
                break;

            case VALIDATE:
                double err = profileTargetPos - currentPos;
                integralSum += err * dt;
                double maxInt = (kI > 0.001) ? (0.5 / kI) : 0.0;
                integralSum = Range.clip(integralSum, -maxInt, maxInt);

                double deriv = (currentPos - lastPosition) / dt;
                lastPosition = currentPos;

                double out = (kP * err) + (kI * integralSum) - (kD * deriv) + kG;
                out = Range.clip(out, -maxPower, maxPower);
                slideMotor.setPower(out);

                double direction = Math.signum(profileTargetPos - profileStartPos);
                if (direction > 0 && currentPos > profileTargetPos) {
                    tuneValidatedOvershoot = Math.max(tuneValidatedOvershoot, currentPos - profileTargetPos);
                } else if (direction < 0 && currentPos < profileTargetPos) {
                    tuneValidatedOvershoot = Math.max(tuneValidatedOvershoot, profileTargetPos - currentPos);
                }

                if (tuneTimer.seconds() > 2.5 || (Math.abs(err) < 0.1 && Math.abs(currentVel) < 0.2)) {
                    if (tuneValidatedOvershoot > 0.5) {
                        kP *= 0.7;
                        kI *= 0.7;
                        kD *= 0.8;
                    }
                    slideMotor.setPower(0);
                    tuneState = TuneState.DONE;
                }
                break;

            case DONE:
            case IDLE:
                break;
        }
    }

    /* ================= TELEMETRY & GETTERS ================= */
    public double getCurrentPositionInches() {
        return slideMotor.getCurrentPosition() / TICKS_PER_INCH;
    }

    public double getCurrentVelocityInches() {
        // Manual calculation is more reliable than motor.getVelocity()
        return (getCurrentPositionInches() - lastPosition) / 0.02; // approx
    }

    public double getTargetPosition() {
        return profileActive ? profileTargetPos : tuneTargetPos;
    }

    public double getError() {
        if (profileActive) return profileTargetPos - getCurrentPositionInches();
        if (isTuning()) return tuneTargetPos - getCurrentPositionInches();
        return 0.0;
    }

    public boolean isBusy() {
        return profileActive || isTuning();
    }

    public boolean isAtTarget() {
        return Math.abs(getError()) < 0.15 && Math.abs(getCurrentVelocityInches()) < 0.5;
    }

    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkV() { return kV; }
    public double getkS() { return kS; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Slide Pos",  "%.2f in", getCurrentPositionInches());
        telemetry.addData("Slide Target", "%.2f in", getTargetPosition());
        telemetry.addData("Slide Error",  "%.3f in", getError());
        telemetry.addData("Slide Power",  "%.3f", slideMotor.getPower());
        telemetry.addLine()
                .addData("kP", "%.4f", kP)
                .addData("kI", "%.4f", kI)
                .addData("kD", "%.4f", kD)
                .addData("kV", "%.4f", kV)
                .addData("kS", "%.4f", kS);
        if (isTuning()) {
            telemetry.addData("TUNER", "%s | cycles=%d", tuneState, tuneCycleCount);
        }
    }

    public void stop() {
        cancelMotion();
        slideMotor.setPower(0);
    }
}