package org.firstinspires.ftc.teamcode.modernTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.hardwareClasses.Box2Extension;

/**
 * Manual test for Box2Extension (cascade stage 1 only).
 * A / B set the target to 0" / 5" so the P gain can be tuned by watching how it settles.
 */
@TeleOp(name = "Box2 Extension Test", group = "Testing")
public class Box2ExtensionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Box2Extension box2 = new Box2Extension(hardwareMap, RobotConfig.BOX2_EXTENSION_MOTOR, true);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                box2.setTargetInches(0.0);
            } else if (gamepad1.b) {
                box2.setTargetInches(9.25);
            } else if (gamepad1.x) {
                box2.setTargetInches(4);
            }

            box2.update();
            box2.addTelemetry(telemetry);
            telemetry.update();
        }

        box2.stop();
    }
}
