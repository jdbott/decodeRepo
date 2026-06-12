package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Flywheel {

    private final DcMotorEx flywheelTop;      // no encoder (or ignored)
    private final DcMotorEx flywheelBottom;   // encoder motor (USED logically by your current setup)
    private final VoltageSensor battery;

    // Target velocity in rad/s
    private double targetVelocity = 0.0;

    // Proportional gain
    private double kP = 0.0019;

    // Feedforward parameters
    // power_ff = (kV * ω + kS) * (V_tuned / V_batt)
    private double kV = 0.0016;
    private double kS = 0.0988;
    private double tunedVoltage = 13.726;

    // Encoder conversion
    private static final double TICKS_PER_REV = 28.0;
    private static final double RAD_PER_TICK = 2.0 * Math.PI / TICKS_PER_REV;

    // Faster velocity filter: exponential moving average
    // Higher alpha = more responsive, lower alpha = smoother
    private double velocityFilterAlpha = 0.38;
    private double smoothedVelocity = 0.0;
    private boolean firstVelocitySample = true;

    // Spin-up bang-bang:
    // If actual velocity is below this fraction of target, slam full power
    private double spinUpBangBangThreshold = 0.93;
    private boolean useBangBang = true;

    // Spin-down zero-power cut:
    // If actual velocity is more than this many rad/s above target, cut power to zero
    private double spinDownCutThresholdRad = 17.0;
    private boolean useSpinDownCut = true;

    /**
     * Default ctor that matches your tuner naming:
     *  - flywheelTop = "shootTop"
     *  - flywheelBottom = "shootBottom"
     *
     * Direction defaults:
     *  - shootTop REVERSE
     *  - shootBottom FORWARD
     */
    public Flywheel(HardwareMap hardwareMap, VoltageSensor battery) {
        this(hardwareMap, battery, RobotConfig.FLYWHEEL_TOP, RobotConfig.FLYWHEEL_BOTTOM, true, false);
    }

    /**
     * Fully configurable ctor.
     *
     * @param topMotorName        hardware name for the top motor
     * @param bottomMotorName     hardware name for the bottom motor
     * @param topMotorReversed    direction for the top motor
     * @param bottomMotorReversed direction for the bottom motor
     */
    public Flywheel(HardwareMap hardwareMap,
                    VoltageSensor battery,
                    String topMotorName,
                    String bottomMotorName,
                    boolean topMotorReversed,
                    boolean bottomMotorReversed) {

        this.battery = battery;

        flywheelTop = hardwareMap.get(DcMotorEx.class, topMotorName);
        flywheelBottom = hardwareMap.get(DcMotorEx.class, bottomMotorName);

        flywheelTop.setDirection(topMotorReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheelBottom.setDirection(bottomMotorReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        flywheelTop.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelBottom.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheelTop.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelBottom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        smoothedVelocity = 0.0;
        firstVelocitySample = true;
    }

    /** Set feedforward values and tuned voltage. */
    public void setFeedforward(double kV, double kS, double tunedVoltage) {
        this.kV = kV;
        this.kS = kS;
        this.tunedVoltage = tunedVoltage;
    }

    /** Set proportional gain. */
    public void setKP(double kP) {
        this.kP = kP;
    }

    /** Set target velocity in rad/s. */
    public void setTargetVelocity(double radPerSec) {
        this.targetVelocity = Math.max(0.0, radPerSec);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Set exponential velocity filter alpha. Suggested range: 0.25 to 0.5 */
    public void setVelocityFilterAlpha(double alpha) {
        this.velocityFilterAlpha = Range.clip(alpha, 0.01, 1.0);
    }

    public double getVelocityFilterAlpha() {
        return velocityFilterAlpha;
    }

    /** Set spin-up bang-bang threshold as a fraction of target. Suggested start: 0.93 */
    public void setSpinUpBangBangThreshold(double threshold) {
        this.spinUpBangBangThreshold = Range.clip(threshold, 0.0, 1.0);
    }

    public double getSpinUpBangBangThreshold() {
        return spinUpBangBangThreshold;
    }

    /** Enable or disable bang-bang spin-up assist. */
    public void setUseBangBang(boolean useBangBang) {
        this.useBangBang = useBangBang;
    }

    public boolean getUseBangBang() {
        return useBangBang;
    }

    /** Set overspeed zero-power cut threshold in rad/s. Suggested start: 17.0 */
    public void setSpinDownCutThresholdRad(double thresholdRad) {
        this.spinDownCutThresholdRad = Math.max(0.0, thresholdRad);
    }

    public double getSpinDownCutThresholdRad() {
        return spinDownCutThresholdRad;
    }

    /** Enable or disable overspeed zero-power cut. */
    public void setUseSpinDownCut(boolean useSpinDownCut) {
        this.useSpinDownCut = useSpinDownCut;
    }

    public boolean getUseSpinDownCut() {
        return useSpinDownCut;
    }

    /** Compute filtered velocity in rad/s. Call frequently. */
    public void updateVelocity() {
        // Intentionally left using flywheelTop because you said the current behavior works
        double velTicksPerSec = flywheelTop.getVelocity();
        double velRadPerSec = Math.abs(velTicksPerSec) * RAD_PER_TICK;

        if (firstVelocitySample) {
            smoothedVelocity = velRadPerSec;
            firstVelocitySample = false;
        } else {
            smoothedVelocity = velocityFilterAlpha * velRadPerSec
                    + (1.0 - velocityFilterAlpha) * smoothedVelocity;
        }
    }

    /** Get filtered velocity in rad/s. */
    public double getVelocityRadPerSec() {
        return smoothedVelocity;
    }

    /** Get filtered velocity in RPM. */
    public double getVelocityRPM() {
        return smoothedVelocity * 60.0 / (2.0 * Math.PI);
    }

    /** Get last applied power from the bottom motor. */
    public double getPower() {
        return flywheelBottom.getPower();
    }

    /**
     * Update motor power using:
     * 1) spin-up bang-bang below target
     * 2) spin-down zero-power cut above target
     * 3) feedforward + P near target
     *
     * Call frequently in loop().
     */
    public void update() {
        updateVelocity();

        if (targetVelocity <= 0.0) {
            flywheelTop.setPower(0.0);
            flywheelBottom.setPower(0.0);
            return;
        }

        double error = targetVelocity - smoothedVelocity;

        // Aggressive spin-up
        if (useBangBang && smoothedVelocity < spinUpBangBangThreshold * targetVelocity) {
            flywheelTop.setPower(-1.0);
            flywheelBottom.setPower(1.0);
            return;
        }

        // Aggressive spin-down
        if (useSpinDownCut && smoothedVelocity > targetVelocity + spinDownCutThresholdRad) {
            flywheelTop.setPower(0.0);
            flywheelBottom.setPower(0.0);
            return;
        }

        // Normal feedforward + P regulation
        double voltage = (battery != null) ? battery.getVoltage() : 13.0;
        double feedforwardPower = (kV * targetVelocity + kS) * (tunedVoltage / voltage);

        double power = feedforwardPower + (kP * error);
        power = Range.clip(power, 0.0, 1.0);

        flywheelTop.setPower(-power);
        flywheelBottom.setPower(power);
    }

    /** Immediately stop flywheel and reset filter state. */
    public void stop() {
        flywheelTop.setPower(0.0);
        flywheelBottom.setPower(0.0);
        smoothedVelocity = 0.0;
        firstVelocitySample = true;
    }
}
