/**
 * This module is responsible for refinements to the odometry-based pose estimation.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

/**
 * Math helper for points and vectors:
 */
class Point {
    public double x, y;
    Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point add(Point other) {
        return new Point(this.x + other.x, this.y + other.y);
    }
    public Point subtract(Point other) {
        return new Point(this.x - other.x, this.y - other.y);
    }
    public Point rotate(double theta) {
        return new Point(Math.cos(theta) * x - Math.sin(theta) * y,
                Math.sin(theta) * x + Math.cos(theta) * y);
    }
    public double dot(Point other) {
        return this.x * other.x + this.y * other.y;
    }
    public double cross(Point other) {
        return this.x * other.y - this.y * other.x;
    }
    public double distance(Point other) {
        return Math.hypot(other.x - this.x, other.y - this.y);
    }
    public double length() {
        return Math.hypot(x, y);
    }
}

/**
 * Math helper for line segments:
 */
class Segment {
    public Point p1, p2;
    public Segment(double x1, double y1, double x2, double y2) {
        p1 = new Point(x1, y1);
        p2 = new Point(x2, y2);
    }
}

/**
 * Math helper for rays:
 */
class Ray {
    public Point origin, direction;
    public Ray(Point origin, Point direction) {
        this.origin = origin;
        this.direction = direction;
    }
}

/**
 * Distance sensor localizer.
 */
class DistanceLocalizer {
    // Description of the distance sensor on the robot:
    static final SensorDescriptor[] SENSOR_DESCRIPTORS = {
        new SensorDescriptor("distance", new Point(-3, -1), Math.PI)
    };

    // This is list is maintained one-to-one with SENSOR_DESCRIPTORS:
    ArrayList<DistanceSensor> sensorHardware = new ArrayList<>();

    // Average distance adder accounting for slope for backdrop intersections:
    static final double BACKDROP_SLOPE_DISTANCE = 2.0;

    // About 30 ms of effective latency for the REV distance sensors:
    static final double LATENCY = 0.030; // 30ms

    // Read a distance sensor at this interval, in seconds:
    static final double READ_INTERVAL = 0.100; // 100ms

    // Maximum distance, in inches, for us to trust a distance measurement:
    static final double MAX_DISTANCE = 48;

    // The measured result needs to be within this many inches of the expected result otherwise
    // it will be discarded. This should be less than the dimension of any robot to account
    // for other robots on the field coming between the sensor and the wall segment:
    static final double CORRECTNESS_THRESHOLD = 12;

    // The sensor's official field-of-view specification is 25 degrees:
    static final double HALF_FOV = Math.toRadians(12.5);

    // Coordinates of the wall segments encircling the field, used for the distance sensor.
    // We leave gaps where the truss supports go.
    static final Segment[] WALL_SEGMENTS = {
        // Clockwise starting in the upper-right corner:
        new Segment(72, 72, 3, 72),
        new Segment(-3, 72, -21, 72), // Top wall in between trusses
        new Segment(-27, 72, -72, 72),

        new Segment(-72, 72, -72, -72), // Left wall

        new Segment(-72, -72, -27, -72),
        new Segment(-21, -72, -3, -72), // Bottom wall in between trusses
        new Segment(3, -72, 72, -72),

        new Segment(72, -72, 72, -48),
        new Segment(60 + BACKDROP_SLOPE_DISTANCE, -48, 60 + BACKDROP_SLOPE_DISTANCE, -24),
        new Segment(72, -24, 72, 24), // Right wall in between backdrops
        new Segment(60 + BACKDROP_SLOPE_DISTANCE, 24, 60 + BACKDROP_SLOPE_DISTANCE, 48),
        new Segment(72, 48, 72, 72)
    };

    // Structure for describing distance sensors on the robot:
    static class SensorDescriptor {
        String name; // Device name of the sensor in the robot configuration:
        Point offset; // Offset in inches from center of rotation, robot facing forward
        double theta; // Orientation of the sensor in radians

        public SensorDescriptor(String name, Point offset, double theta) {
            this.name = name; this.offset = offset; this.theta = theta;
        }
    }

    // Structure for returning results from update():
    static class Result {
        double measurement; // Measured distance
        int sensorIndex; // SENSOR_DESCRIPTORS index
        int segmentIndex; // Target WALL_SEGMENTS index
        double latency; // Latency, in seconds

        // These are for telemetry purposes:
        Point point; // Point computed from the measurement and most recent pose information
        boolean valid; // True if the sensor result was valid and used

        public Result(int sensorIndex, int segmentIndex, double latency) {
            this.sensorIndex = sensorIndex;
            this.segmentIndex = segmentIndex;
            this.latency = latency;
        }
    }

    int currentSensorIndex; // SENSOR_DESCRIPTORS index for the current sensor
    int currentSegmentIndex = -1; // WALL_SEGMENTS index for the current segment, -1 is uninitialized
    double lastReadTime; // Time of the last read of a sensor

    // Create the distance sensor objects:
    DistanceLocalizer(HardwareMap hardwareMap) {
        for (SensorDescriptor descriptor: SENSOR_DESCRIPTORS) {
            sensorHardware.add(hardwareMap.get(DistanceSensor.class, descriptor.name));
        }
    }

    // A method to check if a ray intersects a line segment and return the intersection point or null
    // Courtesy of Copilot.
    public static Point raySegmentIntersection(Ray ray, Segment segment) {
        // Get the vectors of the ray and the segment
        Point r = ray.direction;
        Point s = segment.p2.subtract(segment.p1);

        // Solve the equation: ray.origin + t * r = segment.p1 + u * s
        // If a solution exists, the ray and the segment intersect
        double denominator = r.cross(s);
        // If the denominator is zero, the ray and the segment are parallel
        if (denominator == 0) {
            return null;
        }
        // Otherwise, find the values of t and u
        Point q = segment.p1.subtract(ray.origin);
        double t = q.cross(s) / denominator;
        double u = q.cross(r) / denominator;
        // The ray and the segment intersect if 0 <= t and 0 <= u <= 1
        if (t >= 0 && u >= 0 && u <= 1) {
            // The intersection point is ray.origin + t * r
            return new Point(ray.origin.x + t * r.x, ray.origin.y + t * r.y);
        }
        // Otherwise, there is no intersection
        return null;
    }

    // The pose is used to determine which sensor to use:
    Result update(Pose2d pose) {
        double time = Globals.time();
        if (time - lastReadTime < READ_INTERVAL)
            return null;
        lastReadTime = time;

        // Create an eligibility list of all sensors that point to a wall segment. We'll choose
        // the best of it next.
        ArrayList<Result> eligibleSensors = new ArrayList<>();
        for (int sensorIndex = 0; sensorIndex < SENSOR_DESCRIPTORS.length; sensorIndex++) {
            SensorDescriptor sensor = SENSOR_DESCRIPTORS[sensorIndex];

            // Sensor offset from robot center in field coordinates:
            double sensorAngle = sensor.theta + pose.heading.log();
            Point sensorOffset = sensor.offset.rotate(sensorAngle);

            // Ray representing the sensor's position and orientation on the field:
            Point sensorPoint = new Point(pose.position.x, pose.position.y).add(sensorOffset);
            Point sensorDirection = new Point(Math.cos(sensorAngle), Math.sin(sensorAngle));
            Ray sensorRay = new Ray(sensorPoint, sensorDirection);

            // Find the first wall segment that the sensor is pointing straight at, according to
            // the current pose:
            for (int segmentIndex = 0; segmentIndex < WALL_SEGMENTS.length; segmentIndex++) {
                Segment segment = WALL_SEGMENTS[segmentIndex];

                // Add this sensor and segment to the eligible list if the sensor points at it
                // and the distance is less than MAX_DISTANCE:
                Point hitPoint = raySegmentIntersection(sensorRay, segment);
                if (hitPoint != null) {
                    double distance = sensorPoint.distance(hitPoint);
                    if (distance < MAX_DISTANCE) {
                        eligibleSensors.add(new Result(sensorIndex, segmentIndex, LATENCY));
                    }
                }
            }
        }

        // Choose what sensor to use:
        Result result;
        int eligibleCount = eligibleSensors.size();
        if (eligibleCount == 0) {
            // There are no good candidates so simply do a different sensor than last time:
            currentSensorIndex++;
            if (currentSensorIndex >= SENSOR_DESCRIPTORS.length)
                currentSensorIndex = 0;
            result = new Result(currentSensorIndex, 0, LATENCY);
        } else {
            // If more than one eligible sensor, choose one that's different from the one
            // we used last time:
            result = eligibleSensors.get(0);
            if ((result.sensorIndex == currentSensorIndex) && (eligibleCount > 1))
                result = eligibleSensors.get(1);
        }

        currentSensorIndex = result.sensorIndex;
        currentSegmentIndex = result.segmentIndex;

        // Query the hardware:
        result.measurement = sensorHardware.get(result.sensorIndex).getDistance(DistanceUnit.INCH);
        return result;
    }

    // Get the most recently used wall segment:
    public Segment getCurrentSegment() {
        return (currentSegmentIndex != -1) ? WALL_SEGMENTS[currentSegmentIndex] : null;
    }

    // Given a starting pose and a measured distance, calculate a corrected pose.
    static public Pose2d localize(Pose2d pose, Result distance) {
        boolean valid = false; // Assume failure, we'll convert to success later

        SensorDescriptor sensor = SENSOR_DESCRIPTORS[distance.sensorIndex];
        Segment segment = WALL_SEGMENTS[distance.segmentIndex];

        // Sensor offset from robot center in field coordinates:
        double sensorAngle = sensor.theta + pose.heading.log();
        Point sensorOffset = sensor.offset.rotate(sensorAngle);

        // Create the rays representing the edges of the field-of-view:
        Point sensorPoint = new Point(pose.position.x, pose.position.y).add(sensorOffset);
        Point sensorDirection1 = new Point(Math.cos(sensorAngle - HALF_FOV),
                                           Math.sin(sensorAngle - HALF_FOV));
        Point sensorDirection2 = new Point(Math.cos(sensorAngle + HALF_FOV),
                                           Math.sin(sensorAngle + HALF_FOV));
        Ray sensorRay1 = new Ray(sensorPoint, sensorDirection1);
        Ray sensorRay2 = new Ray(sensorPoint, sensorDirection2);

        // Use the results only if the entire field-of-view is contained within the target
        // segment:
        if ((raySegmentIntersection(sensorRay1, segment) != null) &&
            (raySegmentIntersection(sensorRay2, segment) != null)) {

            Point sensorDirection = new Point(Math.cos(sensorAngle), Math.sin(sensorAngle));
            Ray sensorRay = new Ray(sensorPoint, sensorDirection);

            Point hitPoint = raySegmentIntersection(sensorRay, segment);
            assert(hitPoint != null);
            Point hitVector = new Point(pose.position.x, pose.position.y).subtract(hitPoint);
            double expectedDistance = hitVector.length();

            // Ignore measurements that are much shorter than expected (as can happen when
            // another robot blocks the view) or much farther:
            if (Math.abs(expectedDistance - distance.measurement) < CORRECTNESS_THRESHOLD) {
                // Yay, it looks like a valid result. Push the pose out, or pull it in,
                // accordingly:
                double fixupFactor = distance.measurement / expectedDistance;
                Point fixupVector = new Point(hitVector.x * fixupFactor, hitVector.y * fixupFactor);
                Point newPosition = hitPoint.add(fixupVector);

                // @@@@@@ pose = new Pose2d(new Vector2d(newPosition.x, newPosition.y), pose.heading);
                valid = true;
            }
        }

        // Update telemetry:
        distance.valid = valid;
        distance.point = new Point(
                pose.position.x + sensorOffset.x + distance.measurement * Math.cos(sensorAngle),
                pose.position.y + sensorOffset.y + distance.measurement * Math.sin(sensorAngle));

        return pose;
    }
}

/**
 * Localizer for April Tags.
 */
class AprilTagLocalizer {
    // We arbitrarily decide that a pose has to be within this many inches to be reliable:
    final double FINE_EPSILON = 5.0;
    final double COARSE_EPSILON = 10.0;

    static final double HEADING_WINDOW_DURATION = 20.0; // Seconds
    static final int CONFIDENCE_THRESHOLD_COUNT = 10; // Need 10 excellent readings before being confident

    private String poseStatus = ""; // UI status message that is persisted
    private double excellentPoseCount = 0; // Active count of trusted headings so far

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    final Location[] tagLocations = {
            new Location(1, 62.875, 42.750, 180, false), // Blue left backdrop, small
            new Location(2, 62.875, 36.625, 180, false), // Blue middle backdrop, small
            new Location(3, 62.875, 30.625, 180, false), // Blue right backdrop, small
            new Location(4, 62.875, -30.625, 180, false), // Red left backdrop, small
            new Location(5, 62.875, -36.750, 180, false), // Red middle backdrop, small
            new Location(6, 62.875, -42.625, 180, false), // Red right backdrop, small
            new Location(7, -72, -43.0, 0, true),   // Red audience wall, large
            new Location(8, -72, -37.5, 0, false),  // Red audience wall, small
            new Location(9, -72, 37.5, 0, false),  // Blue audience wall, small
            new Location(10, -72, 43.0, 0, true),   // Blue audience wall, large
    };

    static final CameraDescriptor[] CAMERA_DESCRIPTORS = {
            new CameraDescriptor("webcam2", new Point(8.0, -5.75), Math.PI,
                    new Size(1280, 720),
                    906.940247073, 906.940247073, 670.833056673, 355.34234068,
                    0.19) // Seconds
    };

    // Structure for describing cameras on the robot:
    static class CameraDescriptor {
        String name; // Device name in the robot's configuration
        Point offset; // Offset in inches from center of rotation, robot facing forward
        double theta; // Orientation of the sensor in radians
        Size resolution; // Resolution in pixels
        double fx, fy, cx, cy; // Lens intrinsics; fx = -1 to use system default
        double latency; // End-to-end April Tag processing latency, in seconds

        public CameraDescriptor(String name, Point offset, double theta, Size resolution, double fx, double fy, double cx, double cy, double latency) {
            this.name = name; this.offset = offset; this.theta = theta; this.resolution = resolution;
            this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
            this.latency = latency;
        }
    }

    // Structure defining the location of April Tags:
    static class Location {
        int id;
        double x;
        double y;
        double degrees;
        boolean large;

        Location(int id, double x, double y, double degrees, boolean large) {
            this.id = id; this.x = x; this.y = y; this.degrees = degrees; this.large = large;
        }
    }

    // Structure for returning results from update():
    static class Result {
        Pose2d pose; // Measured result
        int cameraIndex; // CAMERA_DESCRIPTORS index
        double latency; // Latency, in seconds
        boolean isConfident; // True once there are enough excellent poses to be confident

        public Result(Pose2d pose, int cameraIndex, double latency, boolean isConfident) {
            this.pose = pose; this.cameraIndex = cameraIndex; this.latency = latency; this.isConfident = isConfident;
        }
    }

    // Storage for filter data:
    static class FilterStorage {
        double residual;
        double time;

        FilterStorage(double residual) {
            this.residual = residual;
            this.time = Globals.time();
        }
    }

    private LinkedList<FilterStorage> headingResiduals = new LinkedList<>(); // Newest to oldest

    AprilTagLocalizer(HardwareMap hardwareMap) {
        CameraDescriptor descriptor = CAMERA_DESCRIPTORS[0];

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

        if (descriptor.fx != -1.0) {
            processorBuilder.setLensIntrinsics(descriptor.fx, descriptor.fy, descriptor.cx, descriptor.cy);
        }

        aprilTag = processorBuilder.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, descriptor.name));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(descriptor.resolution);

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

    // Close when shutting down:
    void close() {
        visionPortal.close();
    }

    // Normalize an angle to [180, -180):
    static double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle <= -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    // Record an excellent pose for filtering purposes:
    void recordExcellentPose(Pose2d oldPose, Pose2d newPose) {
        double headingResidual = normalizeAngle(newPose.heading.log() - oldPose.heading.log());
        headingResiduals.addFirst(new FilterStorage(headingResidual)); // Newest to oldest
    }

    // Filter the new pose's heading:
    Pose2d filterHeading(Pose2d pose) {
        // Remove all aged-out residuals from the histories:
        double time = Globals.time();
        while ((headingResiduals.size() > 0) && (time - headingResiduals.getLast().time > HEADING_WINDOW_DURATION))
            headingResiduals.removeLast(); // Newest to oldest

        // Compute the averages of all the residuals:
        double aggregateHeaderResidual = 0;
        for (FilterStorage heading : headingResiduals) {
            aggregateHeaderResidual = aggregateHeaderResidual + (Double) heading.residual;
        }

        if (headingResiduals.size() != 0)
            aggregateHeaderResidual = aggregateHeaderResidual / headingResiduals.size();

        // Compute how much to correct the current position and heading:
        double headingCorrection = aggregateHeaderResidual;

        // Negatively offset everything in the history to account for the offset we're adding:
        for (FilterStorage heading : headingResiduals) {
            heading.residual = headingCorrection - (double) heading.residual;
        }

        // Correct the current pose to create the new pose:
        return new Pose2d(pose.position, pose.heading.log() + headingCorrection);
    }

    // Find the tag associated with this detection:
    private Location getTag(AprilTagDetection detection) {
        for (Location t : tagLocations) {
            if (t.id == detection.id)
                return t;
        }
        return null;
    }

    // Compute the robot's field-relative pose from the April-tag-relative pose.
    private Pose2d computeRobotPose(AprilTagDetection detection, Location tag) {
        CameraDescriptor descriptor = CAMERA_DESCRIPTORS[0];

        double dx = detection.ftcPose.x + descriptor.offset.x;
        double dy = detection.ftcPose.y + descriptor.offset.y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        double gamma = -(Math.atan(dx / dy) + Math.toRadians(detection.ftcPose.yaw) + Math.toRadians(tag.degrees));
        double x = tag.x + Math.cos(gamma) * distance;
        double y = tag.y + Math.sin(gamma) * distance;

        double theta = Math.toRadians(detection.ftcPose.yaw) + Math.toRadians(tag.degrees) + descriptor.theta;
        return new Pose2d(new Vector2d(x, y), Math.PI - theta);
    }

    // Interim data structure used to track poses determined by the AprilTags:
    static class VisionPose {
        Pose2d pose; // Field-relative robot pose
        double distance; // Distance from current pose
        Location tag; // Tag that determined this pose

        VisionPose(Pose2d pose, double distance, Location tag) {
            this.pose = pose; this.distance = distance; this.tag = tag;
        }
    }

    // Compute the distance between two poses:
    private double distance(Pose2d a, Pose2d b) {
        return Math.hypot(a.position.x - b.position.x, a.position.y - b.position.y);
    }

    // Update loop for April Tags:
    public Result update(Pose2d currentPose, Poser.HistoryRecord historyRecord) {
        ArrayList<VisionPose> visionPoses = new ArrayList<>();
        double minDistance = Float.MAX_VALUE;
        double pipelineLatency = 0;
        CameraDescriptor descriptor = CAMERA_DESCRIPTORS[0];

        List<AprilTagDetection> tagDetections = aprilTag.getFreshDetections();
        if (tagDetections != null) {
            for (AprilTagDetection detection : tagDetections) {
                pipelineLatency = (nanoTime() - tagDetections.get(0).frameAcquisitionNanoTime) * 10e-6;
                if (detection.metadata != null) {
                    Location tag = getTag(detection);
                    if (tag != null) {
                        Pose2d visionPose = computeRobotPose(detection, tag);
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
        int poseCount = visionPoses.size();

        // We love it when there are 3 accurate poses reasonably when reasonably close to
        // the tag - we always use it to set our current pose, regardless of the state of the
        // old pose, and we trust its heading result:
        if ((poseCount == 3) && (tagDetections.get(0).ftcPose.bearing < 48)) {
            double maxNeighborDistance = 0;

            // Compute the distance between each pose and its neighbors:
            double[] distances = new double[3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    if (i != j) {
                        double distance = distance(visionPoses.get(i).pose, visionPoses.get(j).pose);
                        distances[i] += distance;
                        maxNeighborDistance = Math.max(distance, maxNeighborDistance);
                    }
                }
            }

            // If all three pose estimates are reasonably close, choose the one with the
            // shortest distance to the other two:
            if (maxNeighborDistance <= FINE_EPSILON) {
                double min = Math.min(Math.min(distances[0], distances[1]), distances[2]);
                for (int i = 0; i < 3; i++) {
                    if (distances[i] == min) {
                        excellentPoseCount++;
                        visionPose = visionPoses.get(i);
                        recordExcellentPose(currentPose, visionPose.pose);
                        poseStatus = String.format("Excellent vision 3-pose, min %.2f, max %.2f", min, maxNeighborDistance);
                    }
                }
            }
        }

        // If visionPose is null, we didn't find a perfect pose, so lower our standards a little:
        if ((visionPose == null) && (tagDetections != null)) {
            // Set a default status:
            if (poseCount == 0) {
                poseStatus = "No vision pose found";
            } else if (poseCount == 1) {
                poseStatus = "One inadequate vision pose";
            } else {
                poseStatus = String.format("%d inadequate vision poses", poseCount);
            }
            if ((poseCount == 1) && (minDistance < FINE_EPSILON)) {
                // @@@ Check its size?
                // If only a single tag is in view, adopt its pose only if it's relatively close
                // to our current pose estimate:
                poseStatus = String.format("Good single vision pose, %.2f", visionPoses.get(0).distance);
                visionPose = visionPoses.get(0);
            } else if ((poseCount > 1) && (minDistance < COARSE_EPSILON)) {
                // If multiple tags are in view, choose the closest one to our current pose
                // (reasoning that it's unlikely that *both* are wildly bogus).
                for (VisionPose candidatePose : visionPoses) {
                    if (candidatePose.distance == minDistance) {
                        visionPose = candidatePose;
                        poseStatus = String.format("Good one in %d vision pose, %.2f", visionPoses.size(), candidatePose.distance);
                    }
                }
            }
        }

        // Handle some telemetry and visualization logging:
        Globals.telemetry.addLine(String.format("Vision FPS: %.2f", visionPortal.getFps()));
        Globals.telemetry.addLine(String.format("Vision pose count: %d, ms: %.2f", visionPoses.size(), pipelineLatency));
        Globals.telemetry.addLine(poseStatus);

        for (VisionPose rejectPose: visionPoses) {
            if (rejectPose != visionPose) {
                historyRecord.rejectedPoses.add(rejectPose.pose);
            }
        }

        // Return our results:
        if (visionPose == null)
            return null;

        boolean isConfident = (excellentPoseCount > CONFIDENCE_THRESHOLD_COUNT);
        Pose2d resultPose = filterHeading(visionPose.pose);
        return new Result(resultPose, 0, descriptor.latency, isConfident);
    }
}

/**
 * Master localizer for doing pose estimates. Incorporates odometry, April Tags and distance
 * sensors.
 */
public class Poser {
    // These public fields are updated after every call to update():
    public boolean isConfident; // User-calibrated or enough excellent April-tag poses
    public Pose2d pose; // Current pose
    public PoseVelocity2d velocity; // Current velocity, robot-relative not field-relative

    // Keep the history around for this many seconds:
    private final static double HISTORY_DURATION = 2.0;

    // The historic record:
    private LinkedList<HistoryRecord> history = new LinkedList<>(); // Newest first

    // Component localizers:
    private Localizer odometryLocalizer;
    private DistanceLocalizer distanceLocalizer;
    private AprilTagLocalizer aprilTagLocalizer;

    // Record structure for the history:
    static class HistoryRecord {
        // Time, in seconds, of the original record:
        double time;

        // Odometry twist at this time:
        Twist2d twist;

        // Pose after having applied the twist but before distances
        Pose2d postTwistPose;

        // Distance sensor distances for this time interval, if any:
        ArrayList<DistanceLocalizer.Result> distances = new ArrayList<>();

        // Rejected April Tag poses for telemetry purposes:
        ArrayList<Pose2d> rejectedPoses = new ArrayList<>();
    }

    public Poser(HardwareMap hardwareMap, MecanumDrive drive, Pose2d initialPose) {
        if (initialPose != null) {
            pose = initialPose;
            isConfident = true;
        } else {
            pose = new Pose2d(0, 0, 0);
            isConfident = false;
        }
        velocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

        odometryLocalizer = drive.localizer;
        distanceLocalizer = new DistanceLocalizer(hardwareMap);
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
    }

    // Copy a pose:
    static Pose2d copyPose(Pose2d pose) {
        return new Pose2d(
                new Vector2d(pose.position.x, pose.position.y),
                new Rotation2d(pose.heading.real, pose.heading.imag));
    }

    // Update an April-tag or a distance record for a particular time in the past and then
    // recompute all the history back to the present:
    private void changeHistory(double time, Pose2d newAprilTagPose, DistanceLocalizer.Result newDistance) {
        if (history.size() == 0)
            return; // ===>

        // Find the first record older than the target:
        ListIterator<HistoryRecord> iterator = history.listIterator();
        HistoryRecord iteratorRecord;
        do {
            iteratorRecord = iterator.next();
        } while ((iteratorRecord.time <= time) && (iterator.hasNext()));

        // If the newer record is closer, use that:
        if (iterator.hasPrevious()) {
            double nextTime = iteratorRecord.time;
            iteratorRecord = iterator.previous();
            double currentTime = iteratorRecord.time;
            assert(nextTime > time);
            assert(time <= currentTime);
            if (nextTime - time  < time - currentTime) {
                iteratorRecord = iterator.next();
            }
        }

        // Add the distance record:
        if (newDistance != null) {
            iteratorRecord.distances.add(newDistance);
        }

        // Now replay all of the history records back so we can recompute the poses up to the
        // current time:
        pose = (newAprilTagPose != null) ? newAprilTagPose : iteratorRecord.postTwistPose;
        while (true) {
            // Update the post-odometry pose:
            iteratorRecord.postTwistPose = pose;

            // Apply the distances:
            for (DistanceLocalizer.Result distance: iteratorRecord.distances) {
                pose = DistanceLocalizer.localize(pose, distance);
            }

            // Go to the next newer record:
            if (!iterator.hasPrevious())
                break; // ====>
            iteratorRecord = iterator.previous();

            // Apply the odometry twist:
            pose = pose.plus(iteratorRecord.twist);
        }
    }

    // Draw the history visualizations:
    private void visualize() {
        Canvas c = Globals.canvas;

        // Draw the current distance localizer segment:
        Segment segment = distanceLocalizer.getCurrentSegment();
        if (segment != null) {
            c.setStroke("#800080"); // Purple
            c.strokeLine(segment.p1.x, segment.p1.y, segment.p2.x, segment.p2.y);
        }

        if (history.size() != 0) {
            // Go from oldest to newest:
            ListIterator<HistoryRecord> iterator = history.listIterator(history.size());
            HistoryRecord record = iterator.previous();
            while (true) {
                // Draw the pose trail:
                c.setFill("#3F51B5");
                c.fillRect(record.postTwistPose.position.x - 0.5,
                        record.postTwistPose.position.y - 0.5, 1, 1);

                // Draw the distance sensor results in gray (rejects) and green (accepted):
                if (record.distances.size() != 0) {
                    for (DistanceLocalizer.Result distance : record.distances) {
                        c.setFill((distance.valid) ? "#00ff00" : "#d0d0d0");
                        c.fillRect(distance.point.x - 0.5, distance.point.y - 0.5, 1, 1);
                    }
                }

                // Draw the rejected April Tag poses as little almost-white circles:
                for (Pose2d rejectPose : record.rejectedPoses) {
                    c.setStroke("#e0e0e0");
                    MecanumDrive.drawRobot(Globals.canvas, rejectPose, 3);
                }

                // Go to the next newest record:
                if (!iterator.hasPrevious())
                    break;
                record = iterator.previous();
            }
        }

        // Finally, draw our current pose:
        c.setStroke("#3F51B5");
        MecanumDrive.drawRobot(c, pose, 9);
    }

    // Update the pose estimate:
    public void update() {
        double time = Globals.time();
        Twist2dDual<Time> dualTwist = odometryLocalizer.update();
        velocity = dualTwist.velocity().value();

        // Update the current pose from odometry and create a tracking record:
        HistoryRecord historyRecord = new HistoryRecord();
        historyRecord.time = time;
        historyRecord.twist = dualTwist.value();
        pose = pose.plus(historyRecord.twist);
        historyRecord.postTwistPose = copyPose(pose);
        history.addFirst(historyRecord);

        // Delete old history records:
        while (time - history.getLast().time > HISTORY_DURATION) {
            history.removeLast();
        }

        isConfident = true; // @@@@@@@@@@

        // Process April Tags:
        AprilTagLocalizer.Result aprilTag = aprilTagLocalizer.update(pose, historyRecord);
        if (aprilTag != null) {
            // @@@ changeHistory(time - aprilTag.latency, aprilTag.pose, null);
            isConfident |= aprilTag.isConfident;
        }

        // Process the distance sensor. Only change history if the current pose estimate is
        // reliable enough, otherwise chaos will ensue:
        DistanceLocalizer.Result distance = distanceLocalizer.update(pose);
        if ((isConfident) && (distance != null)) {
            changeHistory(time - distance.latency, null, distance);
        }

        visualize();
    }

    // Close when done:
    void close() {
        aprilTagLocalizer.close();
    }
}
