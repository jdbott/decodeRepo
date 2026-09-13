package org.firstinspires.ftc.teamcode.templates;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.alliance.AllianceMirror;
import org.firstinspires.ftc.teamcode.alliance.AllianceStore;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Starting point for a new teleop: Pedro 3 manual driving with a field/robot-centric toggle and a
 * pose reset. Copy into teles/, rename, remove {@code @Disabled}, then add subsystems.
 */
@Disabled
@TeleOp(name = "Template: TeleOp", group = "Templates")
public class TemplateTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        boolean isRed = AllianceStore.isRed(hardwareMap.appContext);
        PoseFactory p = AllianceMirror.poses(isRed);
        Pose resetPose = p.of(72, 72, 90); // where the robot is placed for a pose reset

        // Field heading the drivers face (Pedro frame): blue looks toward -x, red toward +x
        double driverFacingRad = isRed ? 0 : Math.PI;

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(resetPose);
        // Construct subsystems here.

        boolean fieldCentric = true;
        boolean lastToggle = false;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            if (gamepad1.options && !lastToggle) fieldCentric = !fieldCentric;
            lastToggle = gamepad1.options;

            // Pedro drive powers: +forward, +strafe = left, +turn = counter-clockwise
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (fieldCentric) {
                follower.manual(ManualDrive.fieldCentric(
                        forward, strafe, turn, follower.pose().heading(), -driverFacingRad));
            } else {
                follower.manual(forward, strafe, turn);
            }

            if (gamepad1.share) follower.setPose(resetPose);

            Pose pose = follower.pose();
            telemetry.addData("Drive", fieldCentric ? "field-centric (options toggles)" : "robot-centric (options toggles)");
            telemetry.addData("Pose", "(%.1f, %.1f, %.0f deg)", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.update();
        }
    }
}
