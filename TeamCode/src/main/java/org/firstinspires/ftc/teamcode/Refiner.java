/**
 * This module is responsible for refinements to the odometry-based pose estimation.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Pose refiner. Leverages AprilTags and the distance sensor to improve the accuracy of thse
 * estimated pose.
 */
public class Refiner {
    // True if doing April Tags latency calibration:
    private static final boolean LATENCY_CALIBRATION = true;

    // Camera characteristics:
    private static final String CAMERA_NAME = "webcam2";
    private static final double CAMERA_OFFSET_Y = -8.0; // Camera location on the robot
    private static final double CAMERA_OFFSET_X = 5.75;
    private static final double CAMERA_ORIENTATION = Math.PI;
    private static final int CAMERA_PIXEL_WIDTH = 1280;
    private static final int CAMERA_PIXEL_HEIGHT = 720;
    private static final double CAMERA_FX = 906.940247073; // Set to -1.0 to use FTC defaults
    private static final double CAMERA_FY = 906.940247073;
    private static final double CAMERA_CX = 670.833056673;
    private static final double CAMERA_CY = 355.34234068;

    // Distance sensor characteristics:
    private static final double DISTANCE_SENSOR_OFFSET = 8; // Offset from sensor to center of robot

    // Run-time state:
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DistanceSensor distanceSensor;

    // False if the current robot pose hasn't been properly initialized:
    private boolean reliablePose;

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

    public Refiner(HardwareMap hardwareMap, boolean reliablePose) {
        this.reliablePose = reliablePose;
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        initializeAprilTags(hardwareMap);
    }

    void close() {
        visionPortal.close();
    }

    private void initializeAprilTags(HardwareMap hardwareMap) {
        // Create the AprilTag processor.
         AprilTagProcessor.Builder processorBuilder = new AprilTagProcessor.Builder();
         
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

         if (CAMERA_FX != -1.0) {
             processorBuilder.setLensIntrinsics(CAMERA_FX, CAMERA_FY, CAMERA_CX, CAMERA_CY);
         }

        aprilTag = processorBuilder.build();

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

        builder.setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(CAMERA_PIXEL_WIDTH, CAMERA_PIXEL_HEIGHT));

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
        //visionPortal.stopStreaming();
    }

    private void visualizeDistance(Pose2d odometryPose, Canvas canvas) {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        if (distance >= 0) {
            distance += DISTANCE_SENSOR_OFFSET;

            double theta = odometryPose.heading.log();
            double x = odometryPose.position.x + distance * Math.cos(theta);
            double y = odometryPose.position.y + distance * Math.sin(theta);

            canvas.fillRect(x - 1, y - 1, 3, 3);
        }
    }

    private AprilTagLocation getTag(AprilTagDetection detection) {
        for (AprilTagLocation t: tagLocations) {
            if (t.id == detection.id)
                return t;
        }
        return null;
    }

    /**
     * Compute the robot's field-relative pose from the April-tag-relative pose.
     */
    private Pose2d computeRobotPose(AprilTagDetection detection, AprilTagLocation tag) {
        double dx = detection.ftcPose.x + CAMERA_OFFSET_X;
        double dy = detection.ftcPose.y + CAMERA_OFFSET_Y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        double gamma = -(Math.atan(dx / dy) + Math.toRadians(detection.ftcPose.yaw) + Math.toRadians(tag.degrees));
        double x = tag.x + Math.cos(gamma) * distance;
        double y = tag.y + Math.sin(gamma) * distance;

        double theta = Math.toRadians(detection.ftcPose.yaw) + Math.toRadians(tag.degrees) + CAMERA_ORIENTATION;
        return new Pose2d(new Vector2d(x, y), Math.PI - theta);
    }

    /**
     * Interim data structure used to track poses determined by the AprilTags.
     */
    class VisionPose {
        Pose2d pose; // Field-relative robot pose
        double distance; // Distance from current pose
        AprilTagLocation tag; // Tag that determined this pose
        VisionPose(Pose2d pose, double distance, AprilTagLocation tag) {
            this.pose = pose;
            this.distance = distance;
            this.tag = tag;
        }
    }

    /**
     * Compute the distance between two poses.
     */
    private double distance(Pose2d a, Pose2d b) {
        return Math.hypot(a.position.x - b.position.x, b.position.y - b.position.y);
    }

    /**
     *  Return a refined pose based on April Tags and the distance sensor.
     */
    public Pose2d refinePose(Pose2d currentPose, MecanumDrive drive, Canvas canvas) {

        // We arbitrarily decide that a pose has to be within 6 inches to be reliable:
        final double EPSILON = 6.0;

        // visualizeDistance(currentPose, canvas);

        ArrayList<VisionPose> visionPoses = new ArrayList<>();
        double minDistance = Float.MAX_VALUE;

        List<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    AprilTagLocation tag = getTag(detection);
                    if (tag != null) {
                        Pose2d datedVisionPose = computeRobotPose(detection, tag);
                        Pose2d visionPose = drive.applyTwistHistory(datedVisionPose, 0.6);
                        // @@@@@@@@@@@@@@@ Update lag estimate
                        double distance = Math.hypot(
                                currentPose.position.x - visionPose.position.x,
                                currentPose.position.y - visionPose.position.y);

                        minDistance = Math.min(minDistance, distance);
                        visionPoses.add(new VisionPose(visionPose, distance, tag));
                    }
                }
            }
        }

        // This will be the vision pose that we recommend to update the current pose:
        VisionPose visionPose = null;

        if (LATENCY_CALIBRATION) {
            // When doing April Tags latency calibration, return success as soon as we get
            // a single pose, no matter how bad:
            visionPose = (visionPoses.size() != 0) ? visionPoses.get(0) : null;
        } else if (!reliablePose) {
            double maxDistance = 0;
            // The current pose hasn't been properly initialized yet and is unreliable. Choose
            // a vision pose to that we think is reliable:
            if (visionPoses.size() == 3) {
                // Compute the distance between each pose and its neighbors:
                double[] distances = new double[3];
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        if (i != j) {
                            double distance = distance(visionPoses.get(i).pose, visionPoses.get(j).pose);
                            distances[i] += distance;
                            maxDistance = Math.max(distance, maxDistance);
                        }
                    }
                }

                // If all three pose estimates are reasonably close, choose the one with the
                // shortest distance to the other two:
                if (maxDistance <= EPSILON) {
                    double min = Math.min(Math.min(distances[0], distances[1]), distances[2]);
                    for (int i = 0; i < 3; i++) {
                        if (distances[i] == min)
                            visionPose = visionPoses.get(i);
                    }
                }
            }
        } else {
            // The current pose is a pretty good estimate so propose a refined pose estimate
            // only if we think it's highly reliable:
            if (visionPoses.size() == 1) {
                // If only a single tag is in view, adopt its pose only if it's relatively close
                // to our current pose estimate:
                if (visionPoses.get(0).distance < EPSILON)
                    visionPose = visionPoses.get(0);
            } else if (visionPoses.size() > 1) {
                // If multiple tags are in view, choose the closest one (reasoning that it's unlikely
                // that *both* are wildly bogus).
                for (VisionPose candidatePose : visionPoses) {
                    if (candidatePose.distance == minDistance)
                        visionPose = candidatePose;
                }
            }
        }

        // Return null if there was no reliable vision pose:
        if (visionPose == null)
            return null;

        // Remember that the current pose is now trustworthy:
        reliablePose = true;

        // Only use the position of the refined pose. Keep the old heading because that's quite
        // reliable since it comes from the IMU:
        return new Pose2d(visionPose.pose.position.x, visionPose.pose.position.y, currentPose.heading.log());
    }
}
