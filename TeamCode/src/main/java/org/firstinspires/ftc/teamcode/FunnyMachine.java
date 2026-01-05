package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled

@TeleOp(name = "FunnyMachine")
public class FunnyMachine extends OpMode {

    private DcMotorEx motor;

    // Configure
    private static final String MOTOR_NAME = "rightBack"; // your config name
    private int UP_TICKS = 93;
    private static final double POWER = 1;

    // Toggle state + button edge detect
    private boolean toggledUp = false;
    private boolean aLast = false;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(POWER);
    }

    @Override
    public void loop() {
        UP_TICKS = (int) -gamepad1.left_stick_y * 100;
        boolean aNow = gamepad1.a;

        // Rising edge: toggle only when A transitions false -> true
        if (aNow && !aLast) {
            toggledUp = !toggledUp;

            int target = toggledUp ? UP_TICKS : 0;
            motor.setTargetPosition(target);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(POWER);
        }
        aLast = aNow;

        telemetry.addData("ToggledUp", toggledUp);
        telemetry.addData("Current", motor.getCurrentPosition());
        telemetry.addData("Target", motor.getTargetPosition());
        telemetry.addData("Busy", motor.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}