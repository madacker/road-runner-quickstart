package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class Constants {
    // Camera constants for OpenCV detection
    public static final int CAMERA_IMAGE_WIDTH = 640;
    public static final int CAMERA_IMAGE_HEIGHT = 480;

    // Roi (region of interest) for OpenCV detection
    public static final double xLowerBound = 2.0 / 3;
    public static final double xUpperBound = 1;
    public static final double yLowerBound = 2.0 / 3;
    public static final double yUpperBound = 1;
    public static final Rect roi = new Rect(0, (int) ((2 * CAMERA_IMAGE_HEIGHT) / 3.0), CAMERA_IMAGE_WIDTH, (int) (CAMERA_IMAGE_HEIGHT / 3.0));
    public static final Scalar roiColor = new Scalar(0, 255, 255); // cyan

    // Color ranges for OpenCV detection
    public static final Scalar RED_COLOR_DETECT_MAX_HSV = new Scalar(8, 255, 255);
    public static final Scalar RED_COLOR_DETECT_MIN_HSV = new Scalar(0, 60, 50);
    public static final Scalar BLUE_COLOR_DETECT_MAX_HSV = new Scalar(120, 255, 255);
    public static final Scalar BLUE_COLOR_DETECT_MIN_HSV = new Scalar(110, 60, 50);
    public static final Scalar borderColor = new Scalar(0, 255, 0);  // green

    // Sets blur size for gaussian blur in color detection
    public static final Size BLUR_SIZE = new Size(5, 5);
}

class ColorDetectPipeline extends OpenCvPipeline {
    // matrices in the processing pipeline
    Mat roiMat = new Mat();
    Mat blurredMat = new Mat();
    Mat hsvMat = new Mat();
    Mat filteredMat = new Mat();
    Mat contourMask = new Mat();
    Mat outputMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    MatOfPoint currentContour = new MatOfPoint();
    List<MatOfPoint> offsetContoursList = new ArrayList<>();
    MatOfPoint offsetContour = new MatOfPoint();

    // @@@
    Point targetPoint = new Point(0, 0);
    boolean targetDetected;

    public static final Size BLUR_SIZE = new Size(5, 5);

    @Override
    public Mat processFrame(Mat inputMat) {
        // resize the image to the roi so that stuff like volunteer's shirts aren't detected
        roiMat = inputMat.submat(Constants.roi);

        // blur the image to reduce the impact of noisy pixels
        //   each pixel is "averaged" with its neighboring pixels
        Imgproc.GaussianBlur(roiMat, blurredMat, BLUR_SIZE, 0);

        // convert image to HSV color space, which is better for detecting red and blue colors
        Imgproc.cvtColor(blurredMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        // filter out the range of blue or red colors defined in
        //   Constants.BLUE_COLOR_DETECT_MIN_HSV and Constants.BLUE_COLOR_DETECT_MAX_HSV
        //   or Constants.RED_COLOR_DETECT_MIN_HSV and Constants.RED_COLOR_DETECT_MAX_HSV
        Core.inRange(hsvMat, Constants.BLUE_COLOR_DETECT_MIN_HSV, Constants.BLUE_COLOR_DETECT_MAX_HSV, filteredMat);

        // Clears list from the last loop to use as a empty
        contoursList.clear();

        // create a list of contours surrounding groups of contiguous pixels that were filtered
        Imgproc.findContours(filteredMat, contoursList, contourMask, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // copy original image to output image for drawing overlay on
        roiMat.copyTo(outputMat);

        // if no contours are detected, do nothing
        int maxAreaContourIndex;
        targetPoint.x = -1;
        targetPoint.y = -1;
        targetDetected = false;
        if (contoursList.size() > 0) {
            // iterate through list of contours, find max area contour
            double maxArea = 0.0;
            maxAreaContourIndex = 0;
            for (int i = 0; i < contoursList.size(); i++) {
                currentContour = contoursList.get(i);
                double contourArea = Imgproc.contourArea(currentContour);
                if (contourArea > maxArea) {
                    maxArea = contourArea;
                    maxAreaContourIndex = i;
                }
            }
            targetDetected = true;

            // Draw the max area contour at index maxAreaContourIndex for debugging in scrcpy
            Imgproc.drawContours(outputMat, contoursList, maxAreaContourIndex, Constants.borderColor, 2, -1);

            //   draw rectangular bounding box around the max area contour
            //   and draw circle at the center of rectangular bounding box
            Rect boundingRect = Imgproc.boundingRect(contoursList.get(maxAreaContourIndex));
            double boundHeightX = boundingRect.x + boundingRect.width;
            double boundHeightY = boundingRect.y + boundingRect.height;
            Imgproc.rectangle(outputMat, new Point(boundingRect.x, boundingRect.y), new Point(boundHeightX, boundHeightY), Constants.borderColor, 3, Imgproc.LINE_8, 0);
            targetPoint.x = (int) boundingRect.width / 2.0 + boundingRect.x;
            targetPoint.y = (int) boundingRect.height / 2.0 + boundingRect.y;
            Imgproc.circle(outputMat, targetPoint, 10, Constants.borderColor, Imgproc.LINE_4, -1);
        }

        roiMat.release();

        // See this image on the computer using scrcpy
        return outputMat;
    }

    @Override
    public void onViewportTapped() {
            // @@@@@@@@@@@@@@@ robotCamera.pauseViewport();
            // @@@@@@@@@@@@@@@ robotCamera.resumeViewport();
    }
}

@Disabled
@Autonomous(name = "Repro")
public class Repro extends LinearOpMode {
    final boolean CRASH = false;

    public void runOpMode() {
        AprilTagProcessor aprilTag;
        VisionPortal visionPortal;
        OpenCvCamera robotCamera;
        TimeSplitter aprilInitializeTime = TimeSplitter.create("AprilTags initialization");

        telemetry.addLine("Initializing repro...");
        telemetry.update();

        WebcamName camera = hardwareMap.get(WebcamName.class, "webcam");

        ////////////////////////////////////////////////////////////////////////////////////////////
        // OpenCV

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        // robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"));
        // OR... use internal phone camera
        // phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        robotCamera.setPipeline(new ColorDetectPipeline());

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // startStreaming();
                robotCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        // robotCamera.pauseViewport();
        if (!CRASH) {
            sleep(100);
            robotCamera.stopStreaming();
            robotCamera.closeCameraDevice();
        }

        ////////////////////////////////////////////////////////////////////////////////////////////
        // AprilTag

        aprilInitializeTime.startSplit();
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(camera);
        builder.setCameraResolution(new android.util.Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        aprilInitializeTime.endSplit();

        telemetry.addLine("Done initialization!");
        telemetry.update();

        if (CRASH) {
            robotCamera.stopStreaming();
            robotCamera.closeCameraDevice();
        }

        visionPortal.close();

        TimeSplitter.reportAllResults();
        telemetry.addLine("All done, terminating now!");
        telemetry.update();
    }
}
