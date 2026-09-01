package org.firstinspires.ftc.teamcode.modernTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Intake Hopper Test", group = "Testing")
public class IntakeHopperTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor hopper = hardwareMap.get(DcMotor.class, "hoper");

        waitForStart();

        intake.setPower(1.0);
        hopper.setPower(0.75);

        while (opModeIsActive()) {
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Hopper Power", hopper.getPower());
            telemetry.update();
        }

        intake.setPower(0);
        hopper.setPower(0);
    }
}
