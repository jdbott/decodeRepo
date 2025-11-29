package org.firstinspires.ftc.teamcode.vision.opencv;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallDetectionPipeline extends OpenCvPipeline {

    public static class BallResult {
        public String color;
        public double area;
        public double radius;
        public Point center;
        public double distanceFocal;
        public double distanceRegression;
    }

    private final double realBallDiameterCM = 12.7;
    private Double focalLengthPX = null;
    private final double knownDistanceCM = 45;
    private final double minArea = 300;

    public List<BallResult> detections = new ArrayList<>();

    private final Scalar P_L = new Scalar(120, 43, 45);
    private final Scalar P_U = new Scalar(179, 255, 255);

    private final Scalar G_L = new Scalar(30, 75, 93);
    private final Scalar G_U = new Scalar(88, 255, 255);

    @Override
    public Mat processFrame(Mat frame) {

        detections.clear();

        Mat resized = new Mat();
        Imgproc.resize(frame, resized, new Size(600, (600.0/frame.cols())*frame.rows()));

        Imgproc.GaussianBlur(resized, resized, new Size(11, 11), 0);

        Mat hsv = new Mat();
        Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_BGR2HSV);

        Mat maskPurple = new Mat();
        Core.inRange(hsv, P_L, P_U, maskPurple);

        Imgproc.erode(maskPurple, maskPurple, new Mat(), new Point(-1,-1), 2);
        Imgproc.dilate(maskPurple, maskPurple, new Mat(), new Point(-1,-1), 2);

        detectColor(maskPurple, resized, "Purple");

        Mat maskGreenHSV = new Mat();
        Core.inRange(hsv, G_L, G_U, maskGreenHSV);

        Mat lab = new Mat();
        Imgproc.cvtColor(resized, lab, Imgproc.COLOR_BGR2Lab);
        List<Mat> labChannels = new ArrayList<>();
        Core.split(lab, labChannels);
        Mat A = labChannels.get(1);

        Mat maskLAB = new Mat();
        Imgproc.threshold(A, maskLAB, 135, 255, Imgproc.THRESH_BINARY_INV);

        Mat maskGreen = new Mat();
        Core.bitwise_and(maskGreenHSV, maskLAB, maskGreen);

        Imgproc.morphologyEx(maskGreen, maskGreen, Imgproc.MORPH_OPEN,
                Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        Imgproc.morphologyEx(maskGreen, maskGreen, Imgproc.MORPH_CLOSE,
                Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        detectColor(maskGreen, resized, "Green");

        return resized;
    }

    private void detectColor(Mat mask, Mat display, String colorName) {

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        if (hierarchy.empty()) return;

        for (int i = 0; i < contours.size(); i++) {
            double parent = hierarchy.get(0, i)[3];
            if (parent != -1) continue;
            double area = Imgproc.contourArea(contours.get(i));
            if (area < minArea) continue;

            Point center = new Point();
            float[] radiusArr = new float[1];
            Imgproc.minEnclosingCircle(new MatOfPoint2f(contours.get(i).toArray()), center, radiusArr);

            double radius = radiusArr[0];
            if (radius < 10) continue;

            Scalar color = colorName.equals("Purple") ?
                    new Scalar(255, 0, 255) : new Scalar(0, 255, 0);

            Imgproc.circle(display, center, (int) radius, color, 2);

            BallResult result = new BallResult();
            result.color = colorName;
            result.area = area;
            result.center = center;
            result.radius = radius;

            if (focalLengthPX == null)
                focalLengthPX = calibrateFocalLength(radius);

            result.distanceFocal = computeDistanceFocal(radius);

            double areaNorm = (area / 10000.0);
            result.distanceRegression =
                    (-0.0732667 * Math.pow(areaNorm, 3))
                            + (0.402269 * Math.pow(areaNorm, 2))
                            - (0.807345 * areaNorm)
                            + 0.984157;

            detections.add(result);
        }
    }

    private double calibrateFocalLength(double pixelRadius) {
        double pixelDiameter = 2 * pixelRadius;
        return (pixelDiameter * knownDistanceCM) / realBallDiameterCM;
    }

    private double computeDistanceFocal(double pixelRadius) {
        if (focalLengthPX == null || pixelRadius <= 0)
            return -1;
        double pixelDiameter = 2 * pixelRadius;
        return (realBallDiameterCM * focalLengthPX) / pixelDiameter;
    }
}
