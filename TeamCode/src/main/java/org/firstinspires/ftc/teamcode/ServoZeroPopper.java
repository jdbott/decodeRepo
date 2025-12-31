package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

@TeleOp(name = "ServoTest_P1_P2", group = "Test")
public class ServoZeroPopper extends LinearOpMode {

    private Servo P1;
    private Servo P2;

    @Override
    public void runOpMode() {

        // Initialize servos
        P1 = hardwareMap.get(Servo.class, "P1");
        P2 = hardwareMap.get(Servo.class, "P2");

        // Reverse servos here if needed
        // P1.setDirection(Direction.REVERSE);
        // P2.setDirection(Direction.REVERSE);

        // Init position
//        P1.setPosition(0.07); for pushers
//        P2.setPosition(0.07); and for back ramp

        P1.setPosition(0.0);
        P2.setPosition(0.0);

        telemetry.addLine("Servos initialized to 0.05");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Start position
            P1.setPosition(0.4);
            P2.setPosition(0.4);

            telemetry.addLine("Servos set to 0.4");
            telemetry.update();
        }

        // Keep op mode alive
        while (opModeIsActive()) {
            idle();
        }
    }
}