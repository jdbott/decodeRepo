package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled

@TeleOp(name = "CR Servo Sync Test", group = "Test")
public class CRServoSyncTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        CRServo servo1 = hardwareMap.get(CRServo.class, "CRServo1");
        CRServo servo2 = hardwareMap.get(CRServo.class, "CRServo2");
        servo2.setDirection(CRServo.Direction.REVERSE);

        telemetry.addLine("CR Servo Sync Test");
        telemetry.addLine("Press ▶ to start. Observe direction and speed.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Step 1: forward test
        telemetry.addLine("Forward 50% power");
        telemetry.update();
        servo1.setPower(0.5);
        servo2.setPower(0.5);
        sleep(2000);

        // Step 2: stop
        telemetry.addLine("Stop (should hold still)");
        telemetry.update();
        servo1.setPower(0);
        servo2.setPower(0);
        sleep(1000);

        // Step 3: reverse test
        telemetry.addLine("Reverse 50% power");
        telemetry.update();
        servo1.setPower(-0.5);
        servo2.setPower(-0.5);
        sleep(2000);

        // Step 4: stop again
        telemetry.addLine("Stop");
        telemetry.update();
        servo1.setPower(0);
        servo2.setPower(0);
        sleep(1000);

        // Step 5: fine low-speed test
        telemetry.addLine("Low-speed forward (0.1)");
        telemetry.update();
        servo1.setPower(0.1);
        servo2.setPower(0.1);
        sleep(2000);

        telemetry.addLine("Low-speed reverse (-0.1)");
        telemetry.update();
        servo1.setPower(-0.1);
        servo2.setPower(-0.1);
        sleep(2000);

        // Stop final
        servo1.setPower(0);
        servo2.setPower(0);
        telemetry.addLine("Test complete. Check direction and drift visually.");
        telemetry.update();
        sleep(3000);
    }
}