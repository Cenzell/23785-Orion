package org.firstinspires.ftc.teamcode.BigArm.lib.Pipe;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.*;

public class sampletest extends OpenCvPipeline {

    static Scalar redLow1 = new Scalar(0, 100, 100);
    static Scalar redHigh1 = new Scalar(10, 255, 255);
    static Scalar redLow2 = new Scalar(170, 100, 100);
    static Scalar redHigh2 = new Scalar(179, 255, 255);
    static Scalar blueLow = new Scalar(100, 100, 100);
    static Scalar blueHigh = new Scalar(130, 255, 255);
    static Scalar neutralLow = new Scalar(20, 100, 100);
    static Scalar neutralHigh = new Scalar(30, 255, 255);

    public enum SampleColor {
        RED, BLUE, NEUTRAL
    }

    public static class DetectedSample {
        public final SampleColor color;
        public final RotatedRect orientedRect;
        public final double area;

        public DetectedSample(SampleColor color, RotatedRect orientedRect, double area) {
            this.color = color;
            this.orientedRect = orientedRect;
            this.area = area;
        }
    }

    private static final Map<SampleColor, Scalar[]> COLOR_BOUNDS = new EnumMap<>(SampleColor.class);
    static {
        COLOR_BOUNDS.put(SampleColor.RED, new Scalar[]{redLow1, redHigh1, redLow2, redHigh2});
        COLOR_BOUNDS.put(SampleColor.BLUE, new Scalar[]{blueLow, blueHigh});
        COLOR_BOUNDS.put(SampleColor.NEUTRAL, new Scalar[]{neutralLow, neutralHigh});
    }

    private static final Map<SampleColor, Scalar> CONTOUR_COLORS = new EnumMap<>(SampleColor.class);
    static {
        CONTOUR_COLORS.put(SampleColor.RED, new Scalar(255, 0, 0));
        CONTOUR_COLORS.put(SampleColor.BLUE, new Scalar(0, 0, 255));
        CONTOUR_COLORS.put(SampleColor.NEUTRAL, new Scalar(255, 255, 0));
    }

    private final Mat hsvMat = new Mat();
    private final Mat thresholdMat = new Mat();
    private final Mat thresholdMat2 = new Mat();
    private final Mat contoursOnFrame = new Mat();

    private double minArea = 15;
    private boolean drawContours = true;

    private final List<DetectedSample> detectedSamples = new ArrayList<>();

    public void setMinArea(double area) {
        this.minArea = area;
    }

    public void setDrawContours(boolean draw) {
        this.drawContours = draw;
    }

    public List<DetectedSample> getDetectedSamples() {
        return Collections.unmodifiableList(detectedSamples);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        input.copyTo(contoursOnFrame);

        detectedSamples.clear();

        for (SampleColor color : SampleColor.values()) {
            processColor(color);
        }

        return contoursOnFrame;
    }

    private void processColor(SampleColor color) {
        Scalar[] bounds = COLOR_BOUNDS.get(color);

        if (color == SampleColor.RED) {
            Core.inRange(hsvMat, bounds[0], bounds[1], thresholdMat);
            Core.inRange(hsvMat, bounds[2], bounds[3], thresholdMat2);

            Core.addWeighted(thresholdMat, 1.0, thresholdMat2, 1.0, 0.0, thresholdMat);
        } else {
            Core.inRange(hsvMat, bounds[0], bounds[1], thresholdMat);
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                detectedSamples.add(new DetectedSample(color, rotatedRect, area));

                if (drawContours) {
                    drawDetectedSample(color, rotatedRect, area);
                }
            }
        }
    }

    private void drawDetectedSample(SampleColor color, RotatedRect rotatedRect, double area) {
        drawRotatedRect(contoursOnFrame, rotatedRect, CONTOUR_COLORS.get(color));
        String label = (int) area + "";
        Point labelPosition = new Point(rotatedRect.center.x - 32, rotatedRect.center.y);
        Imgproc.putText(contoursOnFrame, label, labelPosition, Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, CONTOUR_COLORS.get(color), 2);
    }

    private void drawRotatedRect(Mat image, RotatedRect rotatedRect, Scalar color) {
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);
        for (int i = 0; i < 4; i++) {
            Imgproc.line(image, vertices[i], vertices[(i + 1) % 4], color, 2);
        }
    }
}
