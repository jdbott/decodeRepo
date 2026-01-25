package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayDeque;
import java.util.Deque;

public class FlywheelASG {

    private final DcMotorEx flywheel1;   // no encoder (or ignored)
    private final DcMotorEx flywheel2;   // encoder motor (USED for velocity)
    private final VoltageSensor battery;

    // Target velocity in rad/s
    private double targetVelocity = 0.0;

    // Proportional gain
    private double kP = 0.0017;

    // Feedforward parameters
    // power_ff = (kV * ω + kS) * (V_tuned / V_batt)
    private double kV = 0.0016;
    private double kS = 0.1146;
    private double tunedVoltage = 13.899;

    // Encoder conversion
    // Keep consistent with your tuning code (you used 28 ticks/rev).
    private static final double TICKS_PER_REV = 28.0;
    private static final double RAD_PER_TICK = 2.0 * Math.PI / TICKS_PER_REV;

    // Velocity smoothing buffer (rolling average)
    private final Deque<Double> velBuffer = new ArrayDeque<>();
    private int bufferSize = 18;
    private double smoothedVelocity = 0.0;

    /**
     * Default ctor that matches your tuner naming:
     *  - flywheel1 = "shooter1"
     *  - flywheel2 = "shooter2"  (ENCODER motor used for getVelocity())
     *
     * Direction defaults:
     *  - shooter1 REVERSE
     *  - shooter2 FORWARD
     *
     * This matches your tuner where you used:
     *  shooter1.setPower(-power);
     *  shooter2.setPower(+power);
     *
     * Here, you will call setPower(+power) on BOTH, and the directions handle the inversion.
     */
    public FlywheelASG(HardwareMap hardwareMap, VoltageSensor battery) {
        this(hardwareMap, battery, "shooter1", "shooter2", true, false);
    }

    /**
     * Fully configurable ctor.
     *
     * @param motor1Name        hardware name for motor 1 (no encoder / ignored)
     * @param motor2Name        hardware name for motor 2 (encoder motor USED)
     * @param motor1Reversed    direction for motor 1
     * @param motor2Reversed    direction for motor 2
     */
    public FlywheelASG(HardwareMap hardwareMap,
                                        VoltageSensor battery,
                                        String motor1Name,
                                        String motor2Name,
                                        boolean motor1Reversed,
                                        boolean motor2Reversed) {

        this.battery = battery;

        flywheel1 = hardwareMap.get(DcMotorEx.class, motor1Name);
        flywheel2 = hardwareMap.get(DcMotorEx.class, motor2Name);

        flywheel1.setDirection(motor1Reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(motor2Reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // You can keep both as RUN_WITHOUT_ENCODER while still reading velocity from the encoder motor.
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        velBuffer.clear();
        smoothedVelocity = 0.0;
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

    /** Set rolling average buffer size (>= 1). */
    public void setBufferSize(int size) {
        bufferSize = Math.max(1, size);
        while (velBuffer.size() > bufferSize) velBuffer.removeFirst();
    }

    /** Set target velocity in rad/s. */
    public void setTargetVelocity(double radPerSec) {
        this.targetVelocity = Math.max(0.0, radPerSec);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Compute smoothed velocity in rad/s using ONLY flywheel2 encoder velocity. Call frequently. */
    public void updateVelocity() {
        // getVelocity() returns ticks/sec
        double velTicksPerSec = flywheel2.getVelocity();

        // Match your tuner convention: you did "double velTicks = -flywheel2.getVelocity();"
        // Instead of relying on sign, take absolute value and let motor direction handle physical direction.
        double velRadPerSec = Math.abs(velTicksPerSec) * RAD_PER_TICK;

        velBuffer.addLast(velRadPerSec);
        if (velBuffer.size() > bufferSize) velBuffer.removeFirst();

        smoothedVelocity = velBuffer.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
    }

    /** Get smoothed velocity in rad/s. */
    public double getVelocityRadPerSec() {
        return smoothedVelocity;
    }

    /** Get smoothed velocity in RPM. */
    public double getVelocityRPM() {
        return smoothedVelocity * 60.0 / (2.0 * Math.PI);
    }

    /** Get last applied power (motor2). */
    public double getPower() {
        return flywheel2.getPower();
    }

    /**
     * Update motor power using feedforward + kP control. Call frequently.
     * Applies SAME power command to both motors; motor directions determine actual spin direction.
     */
    public void update() {
        updateVelocity();

        double voltage = (battery != null) ? battery.getVoltage() : 13.0; // fallback
        double feedforwardPower = (kV * targetVelocity + kS) * (tunedVoltage / voltage);

        double error = targetVelocity - smoothedVelocity;
        double power = feedforwardPower + (kP * error);

        // Shooter only: clamp to [0, 1]
        power = Range.clip(power, 0.0, 1.0);

        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    /** Immediately stop flywheel. */
    public void stop() {
        flywheel1.setPower(0.0);
        flywheel2.setPower(0.0);
        velBuffer.clear();
        smoothedVelocity = 0.0;
    }
}