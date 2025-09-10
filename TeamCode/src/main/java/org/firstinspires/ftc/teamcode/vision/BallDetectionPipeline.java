package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallDetectionPipeline extends OpenCvPipeline {

    // HSV bounds
    private final Scalar lower_purple = new Scalar(125, 80, 50);
    private final Scalar upper_purple = new Scalar(155, 255, 255);
    private final Scalar lower_green = new Scalar(30, 80, 50);
    private final Scalar upper_green = new Scalar(80, 255, 255);

    public static class Ball {
        public String color;
        public Point center;
        public float radius;

        public Ball(String color, Point center, float radius) {
            this.color = color;
            this.center = center;
            this.radius = radius;
        }
    }

    private final List<Ball> balls = new ArrayList<>();

    public List<Ball> getBalls() {
        return balls;
    }

    @Override
    public Mat processFrame(Mat input) {
        balls.clear();

        Size size = new Size(600, 480);
        Mat resized = new Mat();
        Imgproc.resize(input, resized, size);

        Imgproc.GaussianBlur(resized, resized, new Size(11, 11), 0);
        Mat hsv = new Mat();
        Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_RGB2HSV);

        Mat maskPurple = new Mat();
        Core.inRange(hsv, lower_purple, upper_purple, maskPurple);
        Imgproc.erode(maskPurple, maskPurple, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(maskPurple, maskPurple, new Mat(), new Point(-1, -1), 2);
        detectColor(maskPurple, "Purple", resized);

        Mat maskGreen = new Mat();
        Core.inRange(hsv, lower_green, upper_green, maskGreen);
        Imgproc.erode(maskGreen, maskGreen, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(maskGreen, maskGreen, new Mat(), new Point(-1, -1), 2);
        detectColor(maskGreen, "Green", resized);

        for (Ball d : balls) {
            Imgproc.circle(resized, d.center, (int) d.radius, new Scalar(0, 255, 0), 2);
            Imgproc.putText(resized, d.color, d.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
        }

        return resized;
    }

    private void detectColor(Mat mask, String color, Mat output) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        if (hierarchy.empty()) return;

        for (int i = 0; i < contours.size(); i++) {
            double[] h = hierarchy.get(0, i);
            if (h[3] != -1) continue;

            float[] radius = new float[1];
            Point center = new Point();
            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
            Imgproc.minEnclosingCircle(contour2f, center, radius);

            if (radius[0] > 10) {
                Moments m = Imgproc.moments(contours.get(i));
                if (m.m00 > 0) {
                    Point centroid = new Point(m.m10 / m.m00, m.m01 / m.m00);
                    balls.add(new Ball(color, centroid, radius[0]));
                }
            }
        }
    }
}
