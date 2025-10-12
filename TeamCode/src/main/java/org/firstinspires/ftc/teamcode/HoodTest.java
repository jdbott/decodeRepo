package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HoodTest", group = "Tests")
public class HoodTest extends LinearOpMode {

    private final HoodKinematics hood = new HoodKinematics();
    private final double[] targets = {40, 35, 30, 0};
    private int idx = 0;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        hood.init(hardwareMap);
        telemetry.addLine("A: cycle 45/50/60/65");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            if (a && !lastA) {
                idx = (idx + 1) % targets.length;
                hood.setHoodAngle(targets[idx]);
            }
            lastA = a;

            telemetry.addData("Target", targets[idx]);
            telemetry.addData("target servo", hood.servoAngle(targets[idx]));
            telemetry.update();
            sleep(20);
        }
    }
}