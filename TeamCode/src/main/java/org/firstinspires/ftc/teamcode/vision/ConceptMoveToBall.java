package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Concept: Move To Ball")
public class ConceptMoveToBall extends LinearOpMode {

    private DcMotor FrontLeft, BackLeft, FrontRight, BackRight;
    OpenCvCamera camera;
    BallDetectionPipeline pipeline;

    public void Init() {
        BackLeft = hardwareMap.get(DcMotor.class, "leftRear");
        FrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        FrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        BackRight = hardwareMap.get(DcMotor.class, "rightRear");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera= OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BallDetectionPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera error" + errorCode);
                telemetry.update();
            }
        });

        waitForStart();
        while (opModeIsActive()) {
            // Use only the getter, no intermediate set
            for (BallDetectionPipeline.Ball ball : pipeline.getBalls()) {
                if (ball != null && ball.color.equals("purple") && ball.distance < 50) {
                    moveDistanceCm(ball.distance, 0.5); // Move to 10 cm away from the ball
                    telemetry.addData("Moving to ball", "at distance %.1f cm", ball.distance);
                    telemetry.update();
                }
            }
        }
    }

    /**
     * Moves the robot forward or backward a certain distance in centimeters.
     * @param distanceCm Distance to move in centimeters (positive = forward, negative = backward)
     * @param power Motor power (0 to 1)
     */
    public void moveDistanceCm(double distanceCm, double power) {
        // Constants: adjust these for your robot
        double wheelDiameterCm = 9.6;
        double countsPerRevolution = 537.7;
        double cmPerRevolution = Math.PI * wheelDiameterCm;
        double countsPerCm = countsPerRevolution / cmPerRevolution;
        int targetCounts = (int)(distanceCm * countsPerCm);

        // Save current positions
        int flStart = FrontLeft.getCurrentPosition();
        int frStart = FrontRight.getCurrentPosition();
        int blStart = BackLeft.getCurrentPosition();
        int brStart = BackRight.getCurrentPosition();

        // Set target positions
        FrontLeft.setTargetPosition(flStart + targetCounts);
        FrontRight.setTargetPosition(frStart + targetCounts);
        BackLeft.setTargetPosition(blStart + targetCounts);
        BackRight.setTargetPosition(brStart + targetCounts);

        // Set to RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);

        // Wait until all motors reach target
        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy() && opModeIsActive()) {
            telemetry.addData("Moving", "Target: %d counts", targetCounts);
            telemetry.update();
        }

        // Stop all motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        // Reset to RUN_USING_ENCODER
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}