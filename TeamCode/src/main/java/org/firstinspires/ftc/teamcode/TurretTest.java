package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TurretTest extends LinearOpMode {
    private Turret turret;

    @Override
    public void runOpMode() {
        turret = new Turret(hardwareMap, "turret", DcMotorEx.Direction.FORWARD, 537.6, 4.5, -90, 90); // Example values
        turret.zeroTurret();
        // Wait for the Play button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            turret.rotateToAngle(-45);
            telemetry.addData("Turret Angle", turret.getCurrentAngle());
            telemetry.addData("Target Angle", -45);
            telemetry.update();
        }
    }
}

