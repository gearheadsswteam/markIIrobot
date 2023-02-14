package org.firstinspires.ftc.teamcode.vision;
import static org.firstinspires.ftc.teamcode.vision.VisionValueStorage.*;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Scalar;
import org.opencv.core.Mat;
import java.util.ArrayList;
public class ColorSignalDetectorPipeline extends OpenCvPipeline {
    int caseDetected = 0;
    Mat process = new Mat();
    ArrayList<MatOfPoint> contours;
    Point[] contourArr;
    @Override
    public Mat processFrame(Mat input) {
        contours = new ArrayList<>();
        int[] maxColor = {0, -1};
        for (int i = 0; i < signalLower.length; i++) {
            contours = new ArrayList<>();
            Rect maxRect = new Rect();
            Imgproc.cvtColor(input, process, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(process, signalLower[i], signalUpper[i], process);
            Imgproc.morphologyEx(process, process, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(process, process, Imgproc.MORPH_CLOSE, new Mat());
            Imgproc.GaussianBlur(process, process, new Size(15, 5), 0);
            Imgproc.findContours(process, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, contours, -1, new Scalar(0, 0, 0));
            for (MatOfPoint contour : contours) {
                contourArr = contour.toArray();
                Rect boundingRect = Imgproc.boundingRect(new MatOfPoint2f(contourArr));
                if (contourArr.length >= 15 && boundingRect.area() > signalMinArea && boundingRect.area() > maxRect.area()) {
                    maxRect = boundingRect.clone();
                }
            }
            if (maxRect.area() > maxColor[0]) {
                maxColor = new int[] {(int) maxRect.area(), i};
            }
            Imgproc.rectangle(input, maxRect, new Scalar(255, 255, 255));
            Imgproc.putText(input, "Color "+ (i + 1) + " Area: " + maxRect.area(), new Point(10, 25 + 30 * i), 0, 0.7, new Scalar(100, 100, 255), 2);

        }
        caseDetected = maxColor[1] + 1;
        if (caseDetected == 0) {
            Imgproc.putText(input, "No Signal Detected", new Point(10, 465), 0, 0.7, new Scalar(100, 100, 255), 2);
        } else {
            Imgproc.putText(input, "Case: " + caseDetected, new Point(10, 465), 0, 0.7, new Scalar(100, 100, 255), 2);
        }
        return input;
    }
    public void end() {
        process.release();
    }
    public int getCaseDetected() {
        return caseDetected;
    }
}
