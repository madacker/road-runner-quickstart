/**
 * This module is responsible for refinements to the odometry-based pose estimation.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.explorations.AprilTagTest;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Refiner {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final double CAMERA_OFFSET_Y = 8.0; // Camera location on the robot
    private final double CAMERA_OFFSET_X = 0.0;

    // Structure defining the location of April Tags:
    class AprilTagLocation {
        int id;
        double x;
        double y;
        double degrees;
        boolean large;
        AprilTagLocation(int id, double x, double y, double degrees, boolean large) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.degrees = degrees;
            this.large = large;
        }
    }

    final AprilTagLocation[] tagLocations = {
            new AprilTagLocation(1,62.875,   42.750, 180, false), // Blue left backdrop, small
            new AprilTagLocation(2,62.875,   36.625, 180, false), // Blue middle backdrop, small
            new AprilTagLocation(3,62.875,   30.625, 180, false), // Blue right backdrop, small
            new AprilTagLocation(4,62.875,  -30.625, 180, false), // Red left backdrop, small
            new AprilTagLocation(5,62.875,  -36.750, 180, false), // Red middle backdrop, small
            new AprilTagLocation(6,62.875,  -42.625, 180, false), // Red right backdrop, small
            new AprilTagLocation(7,  -72, -43.0, 0, true),   // Red audience wall, large
            new AprilTagLocation(8,  -72, -37.5, 0, false),  // Red audience wall, small
            new AprilTagLocation(9,  -72,  37.5, 0, false),  // Blue audience wall, small
            new AprilTagLocation(10, -72,  43.0, 0, true),   // Blue audience wall, large
    };

    Refiner(HardwareMap hardwareMap) {
        initializeAprilTags(hardwareMap);
    }

    void close() {
        visionPortal.close();
    }

    private void initializeAprilTags(HardwareMap hardwareMap) {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            //.setDrawCubeProjection(false)
            //.setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            // ... these parameters are fx, fy, cx, cy.
            .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        // Stop streaming to save processing performance:
        visionPortal.stopStreaming();
    }

    private AprilTagLocation getTag(AprilTagDetection detection) {
        for (AprilTagLocation t: tagLocations) {
            if (t.id == detection.id)
                return t;
        }
        return null;
    }

    private Pose2d computePose(AprilTagDetection detection, AprilTagLocation tag) {
        double dx = detection.ftcPose.x + CAMERA_OFFSET_X;
        double dy = detection.ftcPose.y + CAMERA_OFFSET_Y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        double gamma = -(Math.atan(dx / dy) + Math.toRadians(detection.ftcPose.yaw) + Math.toRadians(tag.degrees));
        double x = tag.x + Math.cos(gamma) * distance;
        double y = tag.y + Math.sin(gamma) * distance;

        double theta = Math.toRadians(detection.ftcPose.yaw) + Math.toRadians(tag.degrees);
        return new Pose2d(new Vector2d(x, y), Math.PI - theta);
    }

    Pose2d refinePose(Pose2d odometryPose, Canvas canvas) {
        Pose2d result = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                AprilTagLocation tag = getTag(detection);
                if (tag != null) {
                    Pose2d pose = computePose(detection, tag);

                    if (canvas != null) {
                        canvas.setStroke(tag.large ? "#00ff00" : "#c0c000");
                        MecanumDrive.drawRobot(canvas, pose, 7);
                    }

                    if (tag.large) {
                        result = pose;
                    }
                }
            }
        }

        return result;
    }
}
