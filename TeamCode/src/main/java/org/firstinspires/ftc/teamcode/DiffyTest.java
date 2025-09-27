package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Diffy Test")
public class DiffyTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Uses your Diffy class with BETA = 18/52 for 52T:18T gearing
        Diffy diffy = new Diffy(hardwareMap, "wristLeft", "wristRight");

        // Midpoints at neutral (both servos 0.5)
        diffy.setMidpoints(0.50, 0.50);

        // If roll turns the wrong way, invert it:
        // diffy.setRollDirection(true);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // 1) Neutral: pitch=90°, roll=0°
        diffy.setPitchRoll(90, 0);
        telemetry.addData("Stage", "Neutral P=90 R=0");
        telemetry.update();
        sleep(1000);

        // 2) Roll +180° at same pitch
        diffy.setPitchRoll(90, 180);
        telemetry.addData("Stage", "Roll to +180 at P=90");
        telemetry.update();
        sleep(1500);

        // 3) Pitch to 45°, roll to +90°
        diffy.setPitchRoll(45, 90);
        telemetry.addData("Stage", "P=45, R=90");
        telemetry.update();
        sleep(1500);

        // Optional: return to neutral
        diffy.setPitchRoll(90, 0);
        telemetry.addData("Stage", "Done");
        telemetry.update();
        sleep(500);
    }
}