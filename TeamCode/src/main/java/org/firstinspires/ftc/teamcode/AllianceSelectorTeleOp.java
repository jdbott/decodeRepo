package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Alliance Selector")
public class AllianceSelectorTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        String currentAlliance = AllianceStore.get(hardwareMap.appContext);
        String newAlliance;

        // Figure out what alliance it would switch to
        if ("RED".equalsIgnoreCase(currentAlliance)) {
            newAlliance = "BLUE";
        } else {
            newAlliance = "RED";
        }

        // Show info during init
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Alliance Selector");
            telemetry.addData("Current Alliance", currentAlliance);
            telemetry.addData("Press Play to Switch To", newAlliance);
            telemetry.update();
        }

        if (isStopRequested()) return;

        // Switch alliance immediately on start
        if ("RED".equalsIgnoreCase(newAlliance)) {
            AllianceStore.setRed(hardwareMap.appContext);
        } else {
            AllianceStore.setBlue(hardwareMap.appContext);
        }

        telemetry.addData("Alliance Switched To", newAlliance);
        telemetry.update();

        requestOpModeStop();
    }
}