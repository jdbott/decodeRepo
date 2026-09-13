package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Box 2 cascade extension, stage 1.
 *
 * One goBILDA 312 RPM Yellow Jacket motor (19.2:1, 537.7 ticks/rev at the output shaft)
 * drives a 40-tooth GT2 (2mm pitch) pulley directly, extending stage 1 of the cascade.
 * Stage 2 rides on the same belt and moves at some multiple of stage 1's travel, but
 * that relationship isn't modeled here — this class only reports and controls stage 1
 * extension in inches.
 *
 * Control is a standalone loop (call update() every OpMode loop) instead of the motor
 * controller's built-in RUN_TO_POSITION, so it starts as a plain P controller with kI/kD
 * already wired in at zero for later tuning.
 */
public class Box2Extension {

    private final DcMotorEx motor;

    // goBILDA 5203/5202 series Yellow Jacket, 19.2:1 ratio -> 312 RPM, 537.7 ticks/rev at the output shaft
    private static final double TICKS_PER_REV = 537.7;

    // 40-tooth GT2 (2mm pitch) pulley mounted directly on the motor's output shaft
    private static final double PULLEY_TEETH = 40.0;
    private static final double GT2_PITCH_MM = 2.0;
    private static final double MM_PER_IN = 25.4;
    private static final double INCHES_PER_REV = (PULLEY_TEETH * GT2_PITCH_MM) / MM_PER_IN;

    private static final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;

    // Start P-only; kI/kD default to 0 until stage 1 is tuned
    private double kP = 0.9;
    private double kI = 0.0;
    private double kD = 0.0;

    private double targetInches = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTimeNanos = 0;

    private double maxPower = 0.65;
    private double minExtensionInches = 0.0;
    private double maxExtensionInches = 10; // placeholder until stage 1 travel is measured on the robot

    private static final double TOLERANCE_INCHES = 0.05;

    public Box2Extension(HardwareMap hardwareMap) {
        this(hardwareMap, "box2ExtensionMotor", false);
    }

    public Box2Extension(HardwareMap hardwareMap, String motorName, boolean reversed) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPGain(double kP) {
        this.kP = kP;
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setLimits(double minInches, double maxInches) {
        this.minExtensionInches = minInches;
        this.maxExtensionInches = maxInches;
    }

    public void setMaxPower(double power) {
        this.maxPower = Range.clip(Math.abs(power), 0.0, 1.0);
    }

    /** Command a new stage 1 extension target, in inches. */
    public void setTargetInches(double inches) {
        targetInches = Range.clip(inches, minExtensionInches, maxExtensionInches);
        integralSum = 0.0;
        lastError = targetInches - getCurrentPositionInches();
        lastTimeNanos = 0;
    }

    /** Call every loop iteration to drive the motor toward the current target. */
    public void update() {
        double error = targetInches - getCurrentPositionInches();

        long now = System.nanoTime();
        double dt = (lastTimeNanos == 0) ? 0.0 : (now - lastTimeNanos) / 1.0e9;
        lastTimeNanos = now;

        integralSum += (dt > 0) ? error * dt : 0.0;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
        lastError = error;

        double power = kP * error + kI * integralSum + kD * derivative;
        motor.setPower(Range.clip(power, -maxPower, maxPower));
    }

    public double getCurrentPositionInches() {
        return motor.getCurrentPosition() / TICKS_PER_INCH;
    }

    public double getTargetInches() {
        return targetInches;
    }

    public double getError() {
        return targetInches - getCurrentPositionInches();
    }

    public boolean isAtTarget() {
        return Math.abs(getError()) < TOLERANCE_INCHES;
    }

    public void stop() {
        targetInches = getCurrentPositionInches();
        motor.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Box2 Target (in)", "%.2f", targetInches);
        telemetry.addData("Box2 Current (in)", "%.2f", getCurrentPositionInches());
        telemetry.addData("Box2 Error (in)", "%.3f", getError());
        telemetry.addData("Box2 Power", "%.3f", motor.getPower());
        telemetry.addData("Box2 At Target", isAtTarget());
    }
}
