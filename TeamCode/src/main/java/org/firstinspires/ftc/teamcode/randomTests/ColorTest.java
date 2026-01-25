package org.firstinspires.ftc.teamcode.randomTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.hardwareClasses.ColorV3;

@Disabled

@TeleOp(name = "ColorV3 Green-Purple Test", group = "Tests")
public class ColorTest extends LinearOpMode {

    private ColorV3 colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = new ColorV3(hardwareMap);

        telemetry.addLine("ColorV3 Green/Purple Test Initialized");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        RevColorSensorV3 raw = hardwareMap.get(RevColorSensorV3.class, "Color");

        while (opModeIsActive()) {
            String detected = colorSensor.proximityAndColor();
            double distance = colorSensor.proximity();

            int red = raw.red();
            int green = raw.green();
            int blue = raw.blue();

            telemetry.addData("Detected", detected);
            telemetry.addData("Proximity (in)", "%.2f", distance);
            telemetry.addData("Raw R", red);
            telemetry.addData("Raw G", green);
            telemetry.addData("Raw B", blue);
            telemetry.update();

            sleep(100); // update ~10 times/sec
        }
    }
}