package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class offLinearSlides {

    private final DcMotorEx slideMotor;

    // goBILDA 5202 Series (5.2:1) = 1150 RPM = 145.1 ticks/rev
    // Change to 537.7 if you have the 19.2:1 (312 RPM) motor
    private static final double TICKS_PER_REV = 145.1;
    private static final double SPOOL_DIAMETER_IN = 1.504;
    private static final double INCHES_PER_REV = Math.PI * SPOOL_DIAMETER_IN;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;

    private double maxExtensionInches = 26.0;  // FIXED: was 24.0, so HIGH=26.0 got clipped
    private double minExtensionInches = 0.0;
    private double maxPower = 1.0;

    public static final double RETRACTED = 0.0;
    public static final double LOW = 8.5;
    public static final double MID = 17.0;
    public static final double HIGH = 26.0;

    public offLinearSlides(HardwareMap hardwareMap) {
        this(hardwareMap, "cool", false);
    }

    public offLinearSlides(HardwareMap hardwareMap, String motorName, boolean reversed) {
        slideMotor = hardwareMap.get(DcMotorEx.class, motorName);
        slideMotor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset and use hardware position control (RELIABLE)
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setMaxPower(double power) {
        this.maxPower = Range.clip(Math.abs(power), 0.0, 1.0);
    }

    public void setLimits(double minInches, double maxInches) {
        this.minExtensionInches = minInches;
        this.maxExtensionInches = maxInches;
    }

    /** Call this BEFORE waitForStart() if up/down are reversed */
    public void reverseMotorDirection() {
        if (slideMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        // Keep zero at bottom
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    /** Manual control. Switches out of position mode. */
    public void setPower(double power) {
        if (slideMotor.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION) {
            slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
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

    /** Set target position in inches. Motor controller handles PID internally. */
    public void setTargetPosition(double inches) {
        inches = Range.clip(inches, minExtensionInches, maxExtensionInches);
        int targetTicks = (int) Math.round(inches * TICKS_PER_INCH);

        slideMotor.setTargetPosition(targetTicks);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(maxPower);
    }

    public void goToPreset(double presetInches) {
        setTargetPosition(presetInches);
    }

    public double getCurrentPositionInches() {
        return slideMotor.getCurrentPosition() / TICKS_PER_INCH;
    }

    public double getTargetPosition() {
        return slideMotor.getTargetPosition() / TICKS_PER_INCH;
    }

    public double getError() {
        return getTargetPosition() - getCurrentPositionInches();
    }

    public boolean isBusy() {
        return slideMotor.isBusy();
    }

    public boolean isAtTarget() {
        return Math.abs(getError()) < 0.15;
    }

    public void stop() {
        slideMotor.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Slide Pos",    "%.2f in", getCurrentPositionInches());
        telemetry.addData("Slide Target",   "%.2f in", getTargetPosition());
        telemetry.addData("Slide Error",    "%.3f in", getError());
        telemetry.addData("Slide Power",    "%.3f", slideMotor.getPower());
        telemetry.addData("Slide isBusy",   isBusy());
    }
}