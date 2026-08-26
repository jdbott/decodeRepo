package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * FTC Deposit Mechanism - Spins/spits balls out into goals.
 * Pure functionality, no telemetry, no fancy features.
 */
public class offSeasonDeposit {

    // --- Hardware ---
    private DcMotorEx spinnerMotor;
    private Servo gateServo; // Optional: holds balls back until ready

    // --- Constants ---
    // Adjust these to match your motor and gearing
    public static final double SPIN_SPEED = 1.0;      // Full power forward
    public static final double REVERSE_SPEED = -0.5;  // Reverse to clear jams
    public static final double STOP_SPEED = 0.0;

    // Gate positions (if using a gate servo)
    public static final double GATE_OPEN = 0.0;
    public static final double GATE_CLOSED = 0.5;

    // --- State ---
    private boolean isRunning = false;
    private boolean gateOpen = false;

    /**
     * Initialize hardware from the hardware map.
     * @param hwMap The hardware map from your OpMode
     * @param motorName Name configured in RC for the spinner motor
     */
    public offSeasonDeposit(HardwareMap hwMap, String motorName) {
        spinnerMotor = hwMap.get(DcMotorEx.class, motorName);
        spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinnerMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Initialize with an optional gate servo.
     * @param hwMap The hardware map
     * @param motorName Spinner motor config name
     * @param servoName Gate servo config name
     */
    public offSeasonDeposit(HardwareMap hwMap, String motorName, String servoName) {
        this(hwMap, motorName);
        gateServo = hwMap.get(Servo.class, servoName);
        closeGate();
    }

    // --- Core Actions ---

    /** Start spinning forward to spit balls out. */
    public void spin() {
        spinnerMotor.setPower(SPIN_SPEED);
        isRunning = true;
    }

    /** Reverse spin to clear jams or unload. */
    public void reverse() {
        spinnerMotor.setPower(REVERSE_SPEED);
        isRunning = true;
    }

    /** Stop the spinner. */
    public void stop() {
        spinnerMotor.setPower(STOP_SPEED);
        isRunning = false;
    }

    // --- Gate Control (if using servo) ---

    /** Open gate to let balls feed into spinner. */
    public void openGate() {
        if (gateServo != null) {
            gateServo.setPosition(GATE_OPEN);
            gateOpen = true;
        }
    }

    /** Close gate to hold balls back. */
    public void closeGate() {
        if (gateServo != null) {
            gateServo.setPosition(GATE_CLOSED);
            gateOpen = false;
        }
    }

    /** Open gate and immediately start spinning. */
    public void fire() {
        openGate();
        spin();
    }

    /** Close gate and stop spinner. */
    public void ceaseFire() {
        closeGate();
        stop();
    }

    // --- State Getters ---

    public boolean isRunning() {
        return isRunning;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    // --- Utility ---

    /** Set raw spinner power (-1.0 to 1.0). */
    public void setPower(double power) {
        spinnerMotor.setPower(power);
        isRunning = power != 0;
    }

    /** Get current spinner power. */
    public double getPower() {
        return spinnerMotor.getPower();
    }
}