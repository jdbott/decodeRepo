package org.firstinspires.ftc.teamcode.vision.limelight;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Pose Testing", group = "Vision")
public class PoseTesting extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.resetYaw();
    }

    public void start(){
        limelight.start();
    }

    public void loop() {
        YawPitchRollAngles orientations = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientations.getYaw());
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

            final double METERS_TO_INCHES = 39.3701;
            final double FIELD_LENGTH_METERS = 3.6576;
            final double HALF_FIELD = FIELD_LENGTH_METERS / 2.0;

            double x_m = botPose.getPosition().x;
            double y_m = botPose.getPosition().y;

            // Shift origin from field center → bottom-left
            double pedroX_m = x_m;
            double pedroY_m = y_m + HALF_FIELD;

            // Convert to inches
            double pedroX_in = pedroX_m * METERS_TO_INCHES;
            double pedroY_in = pedroY_m * METERS_TO_INCHES;

            // Optional: flip Y if Limelight’s +Y points opposite Pedro’s
            //pedroY_in = (FIELD_LENGTH_METERS - pedroY_m) * METERS_TO_INCHES;

            // Offset correction (if Limelight not centered)
            double LL_OFFSET_X = 0.0;
            double LL_OFFSET_Y = 0.0;
            pedroX_in += LL_OFFSET_X;
            pedroY_in += LL_OFFSET_Y;

            telemetry.addData("Pedro X (in)", pedroX_in);
            telemetry.addData("Pedro Y (in)", pedroY_in);
            telemetry.addData("Heading (deg)", orientations.getYaw());
        } else {
            telemetry.addData("Tag Pose", "No target found");
        }
    }

}