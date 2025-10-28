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

@TeleOp(name = "Estimate Distance to Target", group = "Vision")
public class EstimateDistance extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    private static double h1 = 4.0; //height of limelight in inches
    private static double h2 = 26.0;  //height of target in inches
    private static double a1 = 20.0; //angle of limelight
    private double a2;

    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
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

    public void loop(){
        YawPitchRollAngles orientations = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientations.getYaw());

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            a2 = llResult.getTy();
            double distance = (h2 - h1) / Math.tan(Math.toRadians(a1 + a2));

            double botposeDistance = llResult.getBotposeAvgDist();
            telemetry.addData("Estimated Distance (inches)", distance);
            telemetry.addData("Botpose Average Distance (inches)", botposeDistance);
            telemetry.addData("Tag X Angle", llResult.getTx());
            telemetry.addData("Tag Pose Area", llResult.getTa());
        } else {
            telemetry.addData("Estimated Distance", "No target found");

        }
    }
}
