package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ServoController {

    // --- Hardware ---
    private CRServo servo1, servo2;
    private DcMotorEx encoderMotor; // external REV Through Bore encoder

    // --- Encoder tracking ---
    private int zeroTicks = 0;
    private static final double TICKS_PER_REV = 8192.0;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

    // --- Control parameters ---
    private double kP = 0.005;
    private double kD = 0.000005;
    private static final double MAX_OUTPUT = 1.0;

    // --- Target tracking (command-space, 0–360) ---
    private double lastSetTargetDeg = 0.0;  // always [0,360)

    // --- Unwrap / timing state ---
    private double contMeasDeg = 0.0;       // continuous measurement (deg)
    private double lastAbsDeg  = 0.0;       // last absolute [0,360) sample
    private long   lastTimeNanos = 0L;      // for D term timing

    // --- Status ---
    private boolean servosBusy = false;

    // --- Constructor ---
    public ServoController(HardwareMap hw) {
        servo1 = hw.get(CRServo.class, "CRServo1");
        servo2 = hw.get(CRServo.class, "CRServo2");
        servo2.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderMotor = hw.get(DcMotorEx.class, "intakeMotor");
        encoderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        zeroNow();
        lastSetTargetDeg = getAbsoluteAngleDeg(); // initialize command to current
    }

    // --- Configuration ---
    public void setKP(double gain) { kP = gain; }
    public void setKD(double gain) { kD = gain; }

    // --- Zero encoder ---
    public void zeroNow() {
        zeroTicks = encoderMotor.getCurrentPosition();
        stopAll();
        servosBusy = false;

        // reset unwrap/timing
        lastAbsDeg = getAbsoluteAngleDeg(); // [0,360)
        contMeasDeg = 0.0;                  // define current as 0 on the continuous axis
        lastTimeNanos = 0L;

        // keep target coherent with current pose
        lastSetTargetDeg = lastAbsDeg;
    }

    // --- Command absolute (stores exactly what you asked for) ---
    public void moveServosToPosition(double targetOutputDeg) {
        targetOutputDeg = -targetOutputDeg;
        lastSetTargetDeg = norm0to360(targetOutputDeg);
        servosBusy = true;
    }

    // --- Command relative (adds to last commanded, not actual) ---
    public void moveServosByRotation(double deltaOutputDeg) {
        deltaOutputDeg = -deltaOutputDeg;
        lastSetTargetDeg = norm0to360(lastSetTargetDeg + deltaOutputDeg);
        servosBusy = true;
    }

    // --- Main update loop ---
    public void update() {
        // 1) Read absolute angle and unwrap to a continuous measurement
        double absNow = getAbsoluteAngleDeg();               // [0,360)
        double step   = shortestDiffDeg(absNow, lastAbsDeg); // [-180,180], wrap-safe increment
        contMeasDeg  += step;                                // continuous angle
        lastAbsDeg    = absNow;

        // 2) Choose target image nearest to current continuous measurement
        double targetEq = nearestEquivalent(lastSetTargetDeg, contMeasDeg);

        // 3) Error in continuous domain (no wrap flip)
        double errorDeg = targetEq - contMeasDeg;

        // 4) P + D(on measurement velocity)
        long now = System.nanoTime();
        double cmdP = -kP * errorDeg;

        double cmdD = 0.0;
        if (lastTimeNanos != 0L) {
            double dt = (now - lastTimeNanos) / 1e9;
            if (dt > 0.002) {                   // ignore ultra-small dt
                double vel = step / dt;         // deg/s, wrap-safe
                cmdD = -kD * vel;               // oppose motion
            }
        }
        lastTimeNanos = now;

        double cmd = Range.clip(cmdP + cmdD, -MAX_OUTPUT, MAX_OUTPUT);
        servo1.setPower(cmd);
        servo2.setPower(cmd);

        servosBusy = Math.abs(errorDeg) > 2.5;
    }

    // --- Status / Telemetry helpers ---
    public boolean isServosBusy() { return servosBusy; }
    public double getTargetAngleDeg() { return lastSetTargetDeg; }
    public double getAbsoluteAngleDeg() {
        int ticks = encoderMotor.getCurrentPosition() - zeroTicks;
        double angle = (ticks * DEGREES_PER_TICK) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    // --- Helpers ---
    private static double shortestDiffDeg(double target0to360, double current0to360) {
        double d = target0to360 - current0to360;
        if (d > 180) d -= 360;
        else if (d <= -180) d += 360;
        return d;
    }

    private static double norm0to360(double a) {
        double x = a % 360.0;
        if (x < 0) x += 360.0;
        return x;
    }

    // Pick the 360*k image of target0to360 nearest to aroundContinuous
    private static double nearestEquivalent(double target0to360, double aroundContinuous) {
        double k = Math.rint((aroundContinuous - target0to360) / 360.0);
        return target0to360 + 360.0 * k;
    }

    // --- Stop ---
    public void stopAll() {
        servo1.setPower(0);
        servo2.setPower(0);
        servosBusy = false;
    }
}