package org.firstinspires.ftc.teamcode.modernTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lift Test", group = "Testing")
public class LiftTest extends LinearOpMode {

    // goBILDA 5000-series bare motor: 28 ticks/rev at the motor shaft
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    // External reduction: 13-tooth pinion drives 80-tooth hub gear (output slower than motor)
    private static final double PINION_TEETH = 13.0;
    private static final double HUB_TEETH = 80.0;
    private static final double OUTPUT_TICKS_PER_REV = MOTOR_TICKS_PER_REV * (HUB_TEETH / PINION_TEETH);

    // 60-tooth GT2 pulley (2mm pitch) on the same shaft as the hub gear
    private static final double PULLEY_TEETH = 60.0;
    private static final double GT2_PITCH_MM = 2.0;
    private static final double IN_PER_MM = 1.0 / 25.4;
    private static final double INCHES_PER_OUTPUT_REV = (PULLEY_TEETH * GT2_PITCH_MM) * IN_PER_MM;

    private static final double TICKS_PER_INCH = OUTPUT_TICKS_PER_REV / INCHES_PER_OUTPUT_REV;

    private static final double TARGET_INCHES = 12;
    private static final int TARGET_TICKS = (int) Math.round(TARGET_INCHES * TICKS_PER_INCH);

    private static final double kP = 0.02;

    @Override
    public void runOpMode() {
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift1");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            int currentTicks = lift.getCurrentPosition();
            int error = TARGET_TICKS - currentTicks;

            double power = kP * error;
            power = Math.max(-1.0, Math.min(1.0, power));
            lift.setPower(power);

            telemetry.addData("Target (in)", TARGET_INCHES);
            telemetry.addData("Target Ticks", TARGET_TICKS);
            telemetry.addData("Current Ticks", currentTicks);
            telemetry.addData("Current (in)", currentTicks / TICKS_PER_INCH);
            telemetry.addData("Error (ticks)", error);
            telemetry.addData("Power", power);
            telemetry.update();
        }

        lift.setPower(0);
    }
}
