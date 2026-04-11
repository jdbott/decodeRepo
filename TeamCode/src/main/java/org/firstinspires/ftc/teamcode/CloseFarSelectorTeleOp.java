package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Close/Far Selector")
public class CloseFarSelectorTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        String currentMode = AutoStartStore.get(hardwareMap.appContext);
        String newMode;

        if ("FAR".equalsIgnoreCase(currentMode)) {
            newMode = "CLOSE";
        } else {
            newMode = "FAR";
        }

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Close/Far Selector");
            telemetry.addData("Current Auto Start", currentMode);
            telemetry.addData("Press Play to Switch To", newMode);
            telemetry.update();
        }

        if (isStopRequested()) return;

        if ("FAR".equalsIgnoreCase(newMode)) {
            AutoStartStore.setFar(hardwareMap.appContext);
        } else {
            AutoStartStore.setClose(hardwareMap.appContext);
        }

        telemetry.addData("Auto Start Switched To", newMode);
        telemetry.update();

        requestOpModeStop();
    }
}