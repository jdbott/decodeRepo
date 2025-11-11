package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Through Bore Test")
public class ThroughBoreTest extends LinearOpMode {

    private DcMotorEx leftFront;

    // REV Through Bore Encoder specifications
    private static final double TICKS_PER_REV = 8192.0;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motor (we’re just using its encoder)
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            int currentTicks = leftFront.getCurrentPosition();

            // Convert ticks to degrees
            double totalRotationDegrees = currentTicks * DEGREES_PER_TICK;

            // Normalize to 0–360 for the current facing angle
            double currentAngle = totalRotationDegrees % 360.0;
            if (currentAngle < 0) currentAngle += 360.0;

            telemetry.addData("Raw Encoder Ticks", currentTicks);
            telemetry.addData("Current Angle (0–360°)", "%.2f", currentAngle);
            telemetry.addData("Total Rotation (°)", "%.2f", totalRotationDegrees);
            telemetry.addData("Total Rotations", "%.2f", totalRotationDegrees / 360.0);
            telemetry.update();
        }
    }
}