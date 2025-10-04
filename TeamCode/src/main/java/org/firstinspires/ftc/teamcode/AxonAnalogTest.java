package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Heading Tracker", group = "Examples")
public class AxonAnalogTest extends OpMode {
    private Servo turretServo;
    private AnalogInput axonAnalog;

    @Override
    public void init() {
        turretServo = hardwareMap.get(Servo.class, "axonCR");
        axonAnalog = hardwareMap.get(AnalogInput.class, "axonAnalog");
        turretServo.setPosition(0.5);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        turretServo.setPosition(gamepad1.left_stick_x);

        double voltage = axonAnalog.getVoltage();
        double angleDeg = (voltage / 3.3) * 360;

        telemetry.addData("Robot Heading", angleDeg);
        telemetry.addData("Voltage", voltage);
        telemetry.update();
    }
}