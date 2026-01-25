package org.firstinspires.ftc.teamcode.randomTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardwareClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
@Disabled

@TeleOp(name = "Turret Heading Test")
public class TurretHeadingTest extends OpMode {

    private Follower follower;
    private Turret turret;
    private IMU imu;

    // Target in standard math-style coordinates:
    // +X right, +Y forward, 0° = +X
    private static final double TARGET_X = -72;
    private static final double TARGET_Y = 72;

    private MultipleTelemetry tel;

    @Override
    public void init() {
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pedro coordinate frame:
        // 0°   = +X direction
        // 90°  = +Y direction
        // 180° = -X direction
        // -90° = -Y direction
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
        follower.updatePose();
        follower.setMaxPower(1);

        // Turret init
        turret = new Turret();
        turret.init(hardwareMap, "turretMotor", DcMotorSimple.Direction.FORWARD);
        turret.setKP(0.008);
        turret.setKF(0.003);
        turret.setAngle(0);

        // IMU init
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
        imu.resetYaw();

        tel.addLine("Turret Heading Test Initialized");
        tel.update();
    }

    @Override
    public void loop() {

        // Update odometry
        follower.update();

        // Robot pose
        double botX = follower.getPose().getX();
        double botY = follower.getPose().getY();
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());  // standard math frame

        // Vector to target
        double dx = TARGET_X - botX;
        double dy = TARGET_Y - botY;

        // Absolute field angle toward target (0°=+X, CCW positive)
        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));  // correct for math coordinate system

        // Turret angle = field angle - robot heading
        double turretAngleNeeded = normalize180(angleToTarget - robotHeadingDeg);

        // Drive turret
        turret.setAngle(-turretAngleNeeded);
        turret.update();

        // Telemetry
        tel.addData("Robot Pose", "(%.1f, %.1f, %.1f°)", botX, botY, robotHeadingDeg);
        tel.addData("Target", "(%.1f, %.1f)", TARGET_X, TARGET_Y);
        tel.addData("Angle to Target", "%.1f°", angleToTarget);
        tel.addData("Turret Command Angle", "%.1f°", turretAngleNeeded);
        tel.update();
    }

    private double normalize180(double a) {
        a = ((a + 180) % 360 + 360) % 360 - 180;
        return a;
    }
}