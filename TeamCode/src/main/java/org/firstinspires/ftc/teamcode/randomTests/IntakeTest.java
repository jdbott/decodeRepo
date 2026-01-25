package org.firstinspires.ftc.teamcode.randomTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled

@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        double basePower = 1.0;
        double stallCurrentThreshold = 5.0; // adjust for your motor
        double stallTimeThreshold = 0.33;   // seconds
        double recoveryDelay = 0.5;         // seconds

        double stallStartTime = 0;
        boolean isStalled = false;

        waitForStart();

        while (opModeIsActive()) {
            // Determine direction
            double power = gamepad1.x ? -basePower : basePower;

            double current = motor.getCurrent(CurrentUnit.AMPS);

            // Detect near-stall current
            if (current >= stallCurrentThreshold) {
                if (!isStalled) {
                    stallStartTime = getRuntime();
                    isStalled = true;
                } else if (getRuntime() - stallStartTime > stallTimeThreshold) {
                    // Stop and wait for recovery
                    motor.setPower(0);
                    sleep((long)(recoveryDelay * 1000));
                    isStalled = false;
                    continue;
                }
            } else {
                isStalled = false;
            }

            motor.setPower(power);

            telemetry.addData("Current (A)", current);
            telemetry.addData("Power", power);
            telemetry.addData("Stalled", isStalled);
            telemetry.update();
        }
    }
}
