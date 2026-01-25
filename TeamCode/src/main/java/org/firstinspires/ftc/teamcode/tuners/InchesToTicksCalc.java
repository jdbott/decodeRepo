package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled

@TeleOp
public class InchesToTicksCalc extends LinearOpMode {
    private DcMotor slideMotor;

    @Override
    public void runOpMode() {
        // Initialize the motor
        slideMotor = hardwareMap.get(DcMotor.class, "slides");

        // Set the motor to use encoder mode
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the Play button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Read encoder ticks
            int ticks = slideMotor.getCurrentPosition();

            // Send telemetry data
            telemetry.addData("Encoder Ticks", ticks);
            telemetry.update();
        }
    }
}