package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.tagIds;
import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.tagSize;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagSignalDetectorPipeline extends OpenCvPipeline {
    int caseDetected = 0;
    long tagDetectorPointer = AprilTagDetectorJNI.createApriltagDetector("tag36h11", 3, 1);
    Mat grayscale = new Mat();
    ArrayList<AprilTagDetection> detections;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grayscale, Imgproc.COLOR_RGB2GRAY);
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(tagDetectorPointer, grayscale, tagSize, 400, 400, 320, 240);
        if (!detections.isEmpty() && tagIds.contains(detections.get(0).id)) {
            caseDetected = tagIds.indexOf(detections.get(0).id) + 1;
            Imgproc.rectangle(input, detections.get(0).corners[0], detections.get(0).corners[2], new Scalar(100, 100, 255), 2);
            Imgproc.putText(input, "Case: " + caseDetected, new Point(10, 465), 0, 0.7, new Scalar(100, 100, 255), 2);
            Imgproc.putText(input, "Tag ID: " + detections.get(0).id, new Point(10, 440), 0, 0.7, new Scalar(100, 100, 255), 2);
        } else {
            caseDetected = 0;
            Imgproc.putText(input, "No Signal Detected", new Point(10, 465), 0, 0.7, new Scalar(100, 100, 255), 2);
            if (detections.isEmpty()) {
                Imgproc.putText(input, "No Tag Detected", new Point(10, 440), 0, 0.7, new Scalar(100, 100, 255), 2);
            } else {
                Imgproc.putText(input, "Tag ID: " + detections.get(0).id, new Point(10, 440), 0, 0.75, new Scalar(100, 100, 255), 2);
            }
        }
        return input;
    }
    public void end() {
        grayscale.release();
        AprilTagDetectorJNI.releaseApriltagDetector(tagDetectorPointer);
    }
    public int getCaseDetected() {
        return caseDetected;
    }
}