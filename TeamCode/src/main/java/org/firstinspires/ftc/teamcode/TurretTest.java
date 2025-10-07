package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Turret Test")
public class TurretTest extends LinearOpMode {
    private Turret turret = new Turret();

    @Override
    public void runOpMode() {
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.setKP(0.01);
        turret.setLimits(-180, 180);

        telemetry.addLine("Turret Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // D-pad to adjust angle target
            if (gamepad1.dpad_left) turret.setAngle(-90);
            if (gamepad1.dpad_right) turret.setAngle(90);
            if (gamepad1.dpad_up) turret.setAngle(0);
            if (gamepad1.dpad_down) turret.setAngle(-180);

            // Update control loop
            turret.update();

            telemetry.addData("Target Angle", turret.getTargetAngle());
            telemetry.addData("Current Angle", turret.getCurrentAngle());
            telemetry.addData("Error", turret.getError());
            telemetry.update();
        }
    }
}