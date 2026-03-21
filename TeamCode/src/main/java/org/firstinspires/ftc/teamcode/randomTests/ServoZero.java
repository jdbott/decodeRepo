package org.firstinspires.ftc.teamcode.randomTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled

@TeleOp(name = "ServoZeroTest")
public class ServoZero extends LinearOpMode {

    private Servo armLeft;
    private Servo armRight;

    @Override
    public void runOpMode() {
        // Initialize servo from hardware map
        armLeft = hardwareMap.get(Servo.class, "wristLeft");
        armLeft.setDirection(Servo.Direction.REVERSE);
        armRight = hardwareMap .get(Servo.class, "wristRight");


        // Set position to zero
        armLeft.setPosition(0.5);
        armRight.setPosition(0.5);

        waitForStart();

        armLeft.setPosition(0.);
        armRight.setPosition(0.);

        // Empty loop to hold program until stop
        while (opModeIsActive()) {
            sleep(20);
        }
    }
}