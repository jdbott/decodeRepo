package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallDetectionPipeline extends OpenCvPipeline {

    private final Scalar lower_purple = new Scalar(125, 80, 50);
    private final Scalar upper_purple = new Scalar(155, 255, 255);
    private final Scalar lower_green = new Scalar(30, 75, 93);
    private final Scalar upper_green = new Scalar(88, 255, 255);

    private final double REAL_BALL_DIAMETER = 12.7;
    private final double FOCAL_LENGTH = 622.001;

    public static class Ball {
        public String color;
        public Point center;
        public float radius;
        public double distance;

        public Ball(String color, Point center, float radius, double distance) {
            this.color = color;
            this.center = center;
            this.radius = radius;
            this.distance = distance;
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
        Imgproc.morphologyEx(maskPurple, maskPurple, Imgproc.MORPH_CLOSE,
                Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15)));
        detectColor(maskPurple, "Purple", resized);


        List<Mat> hsvChannels = new ArrayList<>();
        Core.split(hsv, hsvChannels);
        Imgproc.equalizeHist(hsvChannels.get(2), hsvChannels.get(2)); // equalize V
        Core.merge(hsvChannels, hsv);

        Mat maskHSV = new Mat();
        Core.inRange(hsv, lower_green, upper_green, maskHSV);

        Mat lab = new Mat();
        Imgproc.cvtColor(resized, lab, Imgproc.COLOR_RGB2Lab);
        List<Mat> labChannels = new ArrayList<>();
        Core.split(lab, labChannels);
        Mat A = labChannels.get(1);

        Mat maskLab = new Mat();
        Imgproc.threshold(A, maskLab, 135, 255, Imgproc.THRESH_BINARY_INV);

        Mat maskGreen = new Mat();
        Core.bitwise_and(maskHSV, maskLab, maskGreen);

        Imgproc.morphologyEx(maskGreen, maskGreen, Imgproc.MORPH_OPEN,
                Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5)));
        Imgproc.morphologyEx(maskGreen, maskGreen, Imgproc.MORPH_CLOSE,
                Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15)));

        detectColor(maskGreen, "Green", resized);

        for (Ball d : balls) {
            Scalar drawColor = d.color.equals("Purple") ? new Scalar(255, 0, 255) : new Scalar(0, 255, 0);
            Imgproc.circle(resized, d.center, (int) d.radius, drawColor, 2);
            Imgproc.putText(resized, d.color,
                    new Point(d.center.x - 30, d.center.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, drawColor, 2);
            Imgproc.putText(resized, String.format("%.1f cm", d.distance),
                    new Point(d.center.x - 30, d.center.y + 25),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, drawColor, 2);
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
            if (h[3] != -1) continue; // skip nested contours

            float[] radius = new float[1];
            Point center = new Point();
            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
            Imgproc.minEnclosingCircle(contour2f, center, radius);

            if (radius[0] > 10) {
                Moments m = Imgproc.moments(contours.get(i));
                if (m.m00 > 0) {
                    Point centroid = new Point(m.m10 / m.m00, m.m01 / m.m00);

                    double distance = (REAL_BALL_DIAMETER * FOCAL_LENGTH) / (2.0 * radius[0]);

                    balls.add(new Ball(color, centroid, radius[0], distance));
                }
            }
        }
    }
}
