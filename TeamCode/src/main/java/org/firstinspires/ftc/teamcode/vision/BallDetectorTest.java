package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Ball Detector Test", group="Vision")
public class BallDetectorTest extends LinearOpMode {
    OpenCvCamera camera;
    BallDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera= OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "C920"), cameraMonitorViewId);
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
            for (BallDetectionPipeline.Ball d : pipeline.getBalls()) {
                telemetry.addData("Ball", "%s at (%.1f, %.1f), r=%.1f",
                        d.color, d.center.x, d.center.y, d.radius);
            }
            telemetry.update();
        }
    }
}
