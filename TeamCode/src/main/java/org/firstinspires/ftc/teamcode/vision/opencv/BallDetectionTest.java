package org.firstinspires.ftc.teamcode.vision.opencv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.Locale;

@TeleOp(name = "Ball Detection Test", group = "Vision")
public class BallDetectionTest extends LinearOpMode {

    private OpenCvCamera webcam;
    private BallDetectionPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing camera...");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap
                .appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId
        );

        pipeline = new BallDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("CameraError", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Camera ready.");
        telemetry.addLine("Press ▶ to start detecting.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            BallDetectionPipeline.BallResult[] balls = pipeline.detections.toArray(new BallDetectionPipeline.BallResult[0]);

            if (balls.length > 0) {
                telemetry.addData("Ball Count", balls.length);
                for (int i = 0; i < balls.length; i++) {
                    BallDetectionPipeline.BallResult ball = balls[i];
                    telemetry.addLine("=== Ball " + (i + 1) + " ===");
                    telemetry.addData("Color", ball.color);
                    telemetry.addData("Radius (px)", String.format(Locale.US, "%.2f", ball.radius));
                    telemetry.addData("Area (px)", String.format(Locale.US, "%.2f", ball.area));

                    if (ball.center != null) {
                        telemetry.addData("Center", String.format(Locale.US, "(%.1f, %.1f)", ball.center.x, ball.center.y));
                    } else {
                        telemetry.addData("Center", "(null)");
                    }

                    telemetry.addData("Distance (focal)", String.format(Locale.US, "%.2f cm", ball.distanceFocal));
                    telemetry.addData("Distance (regression)", String.format(Locale.US, "%.2f m", ball.distanceRegression));
                }
            } else {
                telemetry.addLine("No ball detected.");
            }

            telemetry.update();

            sleep(50);
        }

        webcam.closeCameraDevice();
    }
}
