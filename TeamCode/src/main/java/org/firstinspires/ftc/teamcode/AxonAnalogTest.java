package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Turret Heading Tracker", group = "Examples")
public class AxonAnalogTest extends OpMode {
    private Follower follower;
    private Servo turretServo;

    private static final double SERVO_MIN_DEG = 0.0;
    private static final double SERVO_MAX_DEG = 355.0;
    private static final double SERVO_RANGE = SERVO_MAX_DEG - SERVO_MIN_DEG;

    private double initialHeadingDeg;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));

        turretServo = hardwareMap.get(Servo.class, "axonCR");
        turretServo.setPosition(0.0);  // robot starts at 0°, turret at 0°
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        // Capture initial heading (in case it's not exactly 0)
        initialHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
    }

    @Override
    public void loop() {
        // field-centric driving
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();

        // current heading
        double currentHeading = Math.toDegrees(follower.getPose().getHeading());
        if (currentHeading < 0) currentHeading += 360;

        // how far the robot has turned from the initial heading
        double deltaHeading = currentHeading - initialHeadingDeg;
        if (deltaHeading < 0) deltaHeading += 360;

        // we want the turret to cancel out this rotation (i.e., rotate the opposite way)
        double turretAbsAngle = (360 - deltaHeading) % 360;

        // only set servo if it's within range
        if (turretAbsAngle <= SERVO_MAX_DEG) {
            double servoPos = turretAbsAngle / SERVO_RANGE;  // map to [0.0, 1.0]
            turretServo.setPosition(servoPos);

            telemetry.addData("Turret Angle", "%.1f°", turretAbsAngle);
            telemetry.addData("Servo Pos", "%.3f", servoPos);
        } else {
            telemetry.addLine("Turret angle out of range — not updating servo.");
        }

        telemetry.addData("Robot Heading", "%.1f°", currentHeading);
        telemetry.update();
    }
}