package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class offLinearSlides {

    private DcMotorEx slideMotor;

    // Mechanical constants — tune for your spool / lead screw
    // goBILDA 435 RPM = 384.5 ticks/rev; 117 RPM = 1425.1 ticks/rev
    private static final double TICKS_PER_REV = 384.5;
    private static final double SPOOL_DIAMETER_IN = 2.0;
    private static final double INCHES_PER_REV = Math.PI * SPOOL_DIAMETER_IN;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;

    // Control tunables
    private double kP = 0.008;
    private double kF = 0.03;
    private double minPower = 0.04;

    // Soft limits
    private double maxExtensionInches = 24.0;
    private double minExtensionInches = 0.0;

    // State
    private double targetPositionInches = 0.0;
    private boolean positionMode = false;
    private boolean slideBusy = false;

    // Presets
    public static final double RETRACTED = 0.0;
    public static final double LOW = 6.0;
    public static final double MID = 12.0;
    public static final double HIGH = 18.0;

    /** Default constructor. Change "linearSlide" to match your robot config name. */
    public offLinearSlides(HardwareMap hardwareMap) {
        this(hardwareMap, "cool", false);
    }

    /** Configurable constructor. */
    public offLinearSlides(HardwareMap hardwareMap, String motorName, boolean reversed) {
        slideMotor = hardwareMap.get(DcMotorEx.class, motorName);
        slideMotor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // =========================
    // Configuration
    // =========================
    public void setKP(double kP) { this.kP = kP; }
    public void setKF(double kF) { this.kF = kF; }
    public void setMinPower(double power) { this.minPower = Math.abs(power); }

    public void setLimits(double minInches, double maxInches) {
        this.minExtensionInches = minInches;
        this.maxExtensionInches = maxInches;
    }

    // =========================
    // Manual control
    // =========================
    public void setPower(double power) {
        positionMode = false;
        slideBusy = false;

        double current = getCurrentPositionInches();

        if ((current >= maxExtensionInches && power > 0) ||
                (current <= minExtensionInches && power < 0)) {
            slideMotor.setPower(0);
            return;
        }

        slideMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void up()   { setPower(1.0); }
    public void down() { setPower(-1.0); }

    // =========================
    // Position control
    // =========================
    public void setTargetPosition(double inches) {
        targetPositionInches = Range.clip(inches, minExtensionInches, maxExtensionInches);
        positionMode = true;
        slideBusy = true;
    }

    public void goToPreset(double presetInches) {
        setTargetPosition(presetInches);
    }

    // =========================
    // Main update loop
    // =========================
    public void update() {
        if (!positionMode) return;

        double current = getCurrentPositionInches();
        double error = targetPositionInches - current;

        if (Math.abs(error) < 0.15) {
            slideMotor.setPower(0);
            slideBusy = false;
            return;
        }

        double power = kP * error;

        if (kF != 0) {
            power += Math.signum(error) * kF;
        }

        if (power > 0 && power < minPower) power = minPower;
        if (power < 0 && power > -minPower) power = -minPower;

        slideMotor.setPower(Range.clip(power, -1.0, 1.0));
        slideBusy = true;
    }

    // =========================
    // Zeroing / reset
    // =========================
    public void zeroSlide() {
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetPositionInches = 0.0;
        positionMode = false;
        slideBusy = false;
    }

    // =========================
    // Accessors
    // =========================
    public double getCurrentPositionInches() {
        return slideMotor.getCurrentPosition() / TICKS_PER_INCH;
    }

    public double getTargetPosition() {
        return targetPositionInches;
    }

    public double getError() {
        return targetPositionInches - getCurrentPositionInches();
    }

    public boolean isBusy() {
        return slideBusy;
    }

    public boolean isAtTarget() {
        return Math.abs(getError()) < 0.15;
    }

    public void stop() {
        slideMotor.setPower(0);
        positionMode = false;
        slideBusy = false;
    }
}