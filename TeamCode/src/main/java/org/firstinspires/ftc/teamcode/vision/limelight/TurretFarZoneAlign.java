package org.firstinspires.ftc.teamcode.vision.limelight;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;

public class TurretFarZoneAlign {
    private Limelight3A limelight;
    private IMU imu;
    private Turret turret;

    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );
        imu.resetYaw();

        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
    }

    public void start(){
        limelight.start();
    }

    public void loop(){
        YawPitchRollAngles orientations = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientations.getYaw());
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            Pose pedroPose =  new Pose(botPose.getPosition().x, botPose.getPosition().y, botPose.getOrientation().getYaw(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            telemetry.addData("Tag Pose X", pedroPose.getX());
            telemetry.addData("Tag Pose Y", pedroPose.getY());
            telemetry.addData("Tag Pose Heading", pedroPose.getHeading());

        } else {
            telemetry.addData("Tag Pose", "No target found");
        }

        double currentAngle = turret.getCurrentAngle();

        //ts does not work rn
        double targetAngle = 10;
        turret.setAngle(targetAngle);

    }
}
