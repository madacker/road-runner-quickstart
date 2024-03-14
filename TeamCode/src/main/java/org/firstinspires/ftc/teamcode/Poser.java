/**
 * This module is responsible for refinements to the odometry-based pose estimation.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.System.nanoTime;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opticaltracking.OpticalTrackingPaa5100;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

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
 * A simple sliding-window filter that averages April Tag pose residuals relative
 * to the odometry poses.
 */
class DistanceFilter {
    static final double WINDOW_DURATION = 0.5; // Seconds
    static final double WATCHDOG_WINDOW_FRACTION = 0.5; // Fraction of the window duration

    LinkedList<Storage> residuals = new LinkedList<>();

    // Storage for filter data:
    static class Storage {
        double residual;
        double time;

        Storage(double residual) {
            this.residual = residual;
            this.time = Globals.time();
        }
    }

    // Filter the result:
    double filter(double time, double measuredDistance, double estimatedDistance) {
        Globals.assertion(Math.abs(estimatedDistance - measuredDistance) < DistanceLocalizer.CORRECTNESS_THRESHOLD);

        // We might have gone back in time so remove any newer results:
        while ((residuals.size() > 0) && (residuals.getFirst().time >= time))
            residuals.removeFirst();

        // If the newest residuals are too stale (meany it's been a long time since we got a
        // new one), toss them all. We don't want to age them out one at a time because that
        // simply biases everything to the most recent one:
        if ((residuals.size() > 0) &&
                (time - residuals.getFirst().time > WINDOW_DURATION * WATCHDOG_WINDOW_FRACTION)) {
            residuals.clear();
        }

        // Remove all residuals that are too old:
        while ((residuals.size() > 0) && (time - residuals.getLast().time > WINDOW_DURATION))
            residuals.removeLast(); // Newest to oldest

        // Add the new residual:
        residuals.addFirst(new Storage(measuredDistance - estimatedDistance)); // Newest to oldest

        // Compute the averages of all the residuals:
        double aggregateHeaderResidual = 0;
        for (Storage residual: residuals) {
            aggregateHeaderResidual = aggregateHeaderResidual + residual.residual;
        }
        if (residuals.size() != 0)
            aggregateHeaderResidual = aggregateHeaderResidual / residuals.size();

        // Compute how much to correct the current position and heading:
        double distanceCorrection = aggregateHeaderResidual;

        // Negatively offset everything in the history to account for the offset we're adding:
        for (Storage residual: residuals) {
            residual.residual -= distanceCorrection;
        }

        // Correct the estimation to create the new result:
        return estimatedDistance + distanceCorrection;
    }

    // Clear the filter history:
    void clear() {
        residuals.clear();
    }
}

/**
 * A simple sliding-window filter that averages April Tag pose residuals relative
 * to the odometry poses.
 */
class AprilTagFilter {
    static final int POSITION_FILTER_COUNT = 8;

    static final double HEADING_WINDOW_DURATION = 20.0; // Seconds
    static final double POSITION_WINDOW_DURATION = 0.500; // Seconds
    static final double WATCHDOG_WINDOW_FRACTION = 0.5; // Fraction of the window duration
    static final int CONFIDENCE_THRESHOLD_COUNT = 10; // Need 10 excellent readings before being confident
    static final double MAX_POSITION_CHANGE_RATE = 18; // Inches per second
    static final double MAX_HEADING_CHANGE_RATE = Math.toRadians(1); // Degrees per second

    private double confidentAprilTagCount = 0; // Active count of trusted headings so far
    private boolean isConfidentFilter; // True if we're now confident in the results coming out

    // The residuals are maintained in newest-to-oldest order:
    public LinkedList<Storage<Double>> headingResiduals = new LinkedList<>();
    public LinkedList<Storage<Point>> positionResiduals = new LinkedList<>();

    public CircleFitter.Circle boundingCircle; // Circle that bounds all position residuals

    // Storage for filter data:
    static class Storage<T> {
        T residual;
        double time;

        Storage(T residual) {
            this.residual = residual;
            this.time = Globals.time();
        }
    }

    // Specify true for isConfident if a starting pose was specified to Poser()
    AprilTagFilter(boolean isConfidentFilter) {
        this.isConfidentFilter = isConfidentFilter;
    }

    // Update the bounding circle given the current set of residuals:
    CircleFitter.Circle getResidualBoundingCircle() {
        ArrayList<Point> points = new ArrayList<>();
        for (int i = 0; i < positionResiduals.size(); i++) {
            points.add(positionResiduals.get(i).residual);
        }
        return CircleFitter.welzl(points);
    }

    // Filter the position portion of the April Tag. This does no averaging but rather simply
    // pulls the current position into the bounding circle of April Tag samples when there is
    // a sufficient count:
    Point filterPosition(@NonNull Point currentPosition, @NonNull Point aprilTagPosition, boolean confidentAprilTag) {

        // Whilst calibrating, ignore non-confident April Tags:
        if ((!isConfidentFilter) && (!confidentAprilTag))
            return currentPosition; // ====>

        Storage<Point> newResidual = new Storage<>(aprilTagPosition.subtract(currentPosition));
        positionResiduals.addFirst(newResidual); // Newest to oldest
        if (positionResiduals.size() > POSITION_FILTER_COUNT) {
            positionResiduals.removeLast();
        }

        // If it's a confident filter, make sure we have a sufficient count. If it's not a
        // confident filter yet, use what we've got:
        if ((isConfidentFilter) && (positionResiduals.size() < POSITION_FILTER_COUNT))
            return currentPosition; // ====>

        // Calculate the bounding circle along with corresponding vectors and length:
        boundingCircle = getResidualBoundingCircle();
        Point center = new Point(boundingCircle.x, boundingCircle.y);
        Point vectorToCenter = center.subtract(currentPosition);
        double distanceToCenter = vectorToCenter.length();
        double correctionDistance = distanceToCenter - boundingCircle.r;

        // If the filter isn't confident yet, pull all the way to the center:
        if (!isConfidentFilter)
            correctionDistance = distanceToCenter;

        // If the current point is already in the circle, the correction distance will be negative:
        if ((correctionDistance <= 0) || (distanceToCenter == 0))
            return currentPosition; // ====>

        // Okay, we're going to pull the current position towards the center of the circle.
        // Calculate the correction:
        Point correctionVector = vectorToCenter.multiply(correctionDistance / distanceToCenter);

        // Negatively offset all of the sample points to account for the offset we're about
        // to add to the current position:
        for (Storage<Point> residual: positionResiduals) {
            residual.residual = residual.residual.subtract(correctionVector);
        }

        return center.add(correctionVector);

        // @@@ Don't worry about going back in time
        // @@@ Remember green vs. yellow residuals for the visualization
        // @@@ Draw the crosshairs for all current April Tags, remove rejects from history
        // @@@ Green bounding circle when correcting the pose, yellow when not
    }

    double filterHeading(double currentHeading, double aprilTagHeading, boolean confidentAprilTag) {
        double time = Globals.time();

        // If the newest residuals are too stale (meaning it's been a long time since we got a
        // new one), toss them all. We don't want to age them out one at a time because that
        // simply biases everything to the most recent one:
        if ((headingResiduals.size() > 0) &&
                (time - headingResiduals.getFirst().time > HEADING_WINDOW_DURATION * WATCHDOG_WINDOW_FRACTION)) {
            headingResiduals.clear();
        }

        // Remove all residuals that are too old:
        while ((headingResiduals.size() > 0) && (time - headingResiduals.getLast().time > HEADING_WINDOW_DURATION))
            headingResiduals.removeLast(); // Newest to oldest

        // Add confident April Tags to the headings history:
        if (confidentAprilTag) {
            double headingResidual = Globals.normalizeAngle(aprilTagHeading - currentHeading);
            headingResiduals.addFirst(new Storage<>(headingResidual)); // Newest to oldest

            confidentAprilTagCount++;
            if (confidentAprilTagCount > CONFIDENCE_THRESHOLD_COUNT)
                isConfidentFilter = true;
        }

        // Compute the averages of all the residuals:
        double aggregateHeaderResidual = 0;
        for (Storage<Double> heading: headingResiduals) {
            aggregateHeaderResidual = aggregateHeaderResidual + heading.residual;
        }
        if (headingResiduals.size() != 0)
            aggregateHeaderResidual = aggregateHeaderResidual / headingResiduals.size();

        // Compute how much to correct the current position and heading:
        double headingCorrection = aggregateHeaderResidual;

        // Negatively offset everything in the history to account for the offset we're adding:
        for (Storage<Double> heading: headingResiduals) {
            heading.residual = heading.residual - headingCorrection;
        }
        return currentHeading + headingCorrection;
    }

    // Filter the prediction and measurements and return a posterior:
    @NonNull Pose2d filter(Pose2d currentPose, Pose2d aprilTagPose, boolean confidentAprilTag) {
        Point position = filterPosition(
                new Point(currentPose.position), new Point(aprilTagPose.position), confidentAprilTag);
        double heading = filterHeading(
                currentPose.heading.log(), aprilTagPose.heading.log(), confidentAprilTag);

        return new Pose2d(position.vector2d(), heading);
    }

    // True if we have high confidence in the results:
    boolean isConfidentFilter() {
        return isConfidentFilter;
    }

    // Output telemetry:
    void telemetry() { // @@@ Remove this
        Globals.telemetry.addLine(String.format("Position residuals: %d, heading residuals: %d",
                positionResiduals.size(), headingResiduals.size()));
    }
}

/**
 * Distance sensor localizer.
 */
class DistanceLocalizer {
    // Description of the distance sensor on the robot:
    public static final DistanceDescriptor[] DISTANCE_SENSOR_DESCRIPTORS = {
        new DistanceDescriptor("distance", new Point(-1, 3.5), Math.PI)
    };

    // Active state for each one of the distance sensors. This is maintained 1-to-1 with
    // DISTANCE_SENSOR_DESCRIPTORS:
    public SensorState[] sensorStates = new SensorState[DISTANCE_SENSOR_DESCRIPTORS.length];

    // Object tracking the state for each sensor:
    static class SensorState {
        int index; // Index for this entry in both 'sensors' and 'DISTANCE_SENSOR_DESCRIPTORS'
        DistanceSensor hardware; // FTC object for the distance sensor
        DistanceDescriptor descriptor; // Associated descriptor
        Segment trackedWall; // The wall the sensor is currently tracking, null if none
        DistanceFilter filter = new DistanceFilter(); // Filter for this sensor
        boolean enabled; // True if enabled in settings

        public SensorState(int index, DistanceSensor hardware, DistanceDescriptor descriptor) {
            this.index = index;
            this.hardware = hardware;
            this.descriptor = descriptor;
        }
    }

    // Average distance adder accounting for slope for backdrop intersections, in inches:
    static final double BACKDROP_SLOPE_DISTANCE = 2.0;

    // About 30 ms of effective latency for the REV distance sensors:
    static final double LATENCY = 0.030; // 30ms

    // Read a distance sensor at this interval, in seconds:
    static final double READ_INTERVAL = 0.100; // 100ms

    // Maximum distance, in inches, for us to trust a distance sensor measurement:
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
    static class DistanceDescriptor {
        String name; // Device name of the sensor in the robot configuration:
        Point offset; // Offset in inches from center of rotation, robot facing forward
        double theta; // Orientation of the sensor in radians

        public DistanceDescriptor(String name, Point offset, double theta) {
            this.name = name; this.offset = offset; this.theta = theta;
        }
    }

    // Structure for returning results from update():
    static class Result {
        double measurement; // Measured distance
        SensorState sensor; // Distance sensor
        Segment wall; // Target wall
        double latency; // Latency, in seconds

        // These are for telemetry purposes:
        Point point; // Point computed from the measurement and most recent pose information
        boolean valid; // True if the sensor result was valid and used

        public Result(SensorState sensor, Segment wall, double latency, double measurement) {
            this.sensor = sensor; this.wall = wall; this.latency = latency;
            this.measurement = measurement;
        }
    }

    SensorState currentSensor; // Currently active sensor, null if none
    Segment currentWall; // Current segment, null if none
    double lastUpdateTime; // Time of the last read of a sensor

    // Create the distance localizer:
    DistanceLocalizer(HardwareMap hardwareMap) {
        // Initialize the sensor state:
        for (int i = 0; i < DISTANCE_SENSOR_DESCRIPTORS.length; i++) {
            DistanceDescriptor descriptor = DISTANCE_SENSOR_DESCRIPTORS[i];
            SensorState state = new SensorState(i, hardwareMap.get(DistanceSensor.class, descriptor.name), descriptor);
            sensorStates[i] = state;
            Settings.registerToggleOption(String.format("Enable %s", descriptor.name),
                    false, enable -> state.enabled = enable );
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

    // Calculate the expected distance from the sensor to the specified wall:
    static public double poseDistanceToWall(Pose2d pose, SensorState sensor, Segment wall) {
        // Sensor offset from robot center in field coordinates:
        double poseHeading = pose.heading.log();
        Point sensorOffset = sensor.descriptor.offset.rotate(poseHeading);
        double sensorAngle = sensor.descriptor.theta + poseHeading;

        // Ray representing the sensor's position and orientation on the field:
        Point sensorPoint = new Point(pose.position).add(sensorOffset);
        Point sensorDirection = new Point(Math.cos(sensorAngle), Math.sin(sensorAngle));
        Ray sensorRay = new Ray(sensorPoint, sensorDirection);

        Point hitPoint = raySegmentIntersection(sensorRay, wall);
        if (hitPoint != null) {
            return sensorPoint.distance(hitPoint);
        }
        return Double.MAX_VALUE;
    }

    // Describes a wall we're tracking and its corresponding sensor:
    static class WallTracker {
        SensorState sensor; // Associated distance sensor
        Segment wall; // Wall the sensor points at, null if none
        double distance; // Estimated distance to the wall, Double.MAX_VALUE if infinite

        public WallTracker(SensorState sensor, Segment wall, double distance) {
            this.sensor = sensor; this.wall = wall; this.distance = distance;
        }
    }

    // Main loop update for the distance sensors. The pose is used to determine which sensor to
    // use next:
    Result update(Pose2d pose, Poser.HistoryRecord record) { // Returns null if no readings
        double time = Globals.time();
        if (time - lastUpdateTime < READ_INTERVAL)
            return null;
        lastUpdateTime = time;

        WallTracker[] wallTrackers = new WallTracker[sensorStates.length];
        for (SensorState sensor: sensorStates) {
            // Set an entry even if the sensor is disabled:
            wallTrackers[sensor.index] = new WallTracker(sensor, null, Double.MAX_VALUE);
            if (sensor.enabled) {
                double poseHeading = pose.heading.log();
                Point sensorOffset = sensor.descriptor.offset.rotate(poseHeading);
                double sensorAngle = sensor.descriptor.theta + poseHeading;

                // Ray representing the sensor's position and orientation on the field:
                Point sensorPoint = new Point(pose.position.x, pose.position.y).add(sensorOffset);
                Point sensorDirection = new Point(Math.cos(sensorAngle), Math.sin(sensorAngle));
                Ray sensorRay = new Ray(sensorPoint, sensorDirection);

                // Find the first wall segment that the sensor is pointing straight at, according to
                // the current pose:
                for (Segment segment : WALL_SEGMENTS) {
                    // Add this sensor and segment to the eligible list if the sensor points at it
                    // and the distance is less than MAX_DISTANCE:
                    Point hitPoint = raySegmentIntersection(sensorRay, segment);
                    if (hitPoint != null) {
                        double distance = sensorPoint.distance(hitPoint);
                        if (distance < Math.min(wallTrackers[sensor.index].distance, MAX_DISTANCE)) {
                            wallTrackers[sensor.index] = new WallTracker(sensor, segment, distance);
                        }
                    }
                }
            }
        }

        // Reset the filters whenever the associated tracking wall changes:
        for (SensorState sensor: sensorStates) {
            if (wallTrackers[sensor.index].wall != null) {
                if (sensor.trackedWall != wallTrackers[sensor.index].wall) {
                    sensor.trackedWall = wallTrackers[sensor.index].wall;
                    sensor.filter.clear();
                }
            }
        }

        // Sort by increasing distance:
        Arrays.sort(wallTrackers, Comparator.comparingDouble(x -> x.distance));

        // Record the walls we're tracking into the history record:
        record.wallTrackers = wallTrackers;

        // Don't waste time querying the sensor if no walls would be tracked:
        if (wallTrackers[0].distance == Double.MAX_VALUE) {
            currentWall = null;
            currentSensor = null;
            return null; // ====>
        }

        // Ping pong between the top two candidates. Always go with the largest unless we did
        // it last time and the second largest is smaller than the maximum:
        int selection = 0;
        if ((wallTrackers.length > 1) &&
            (wallTrackers[1].distance < Double.MAX_VALUE) &&
            (currentSensor != wallTrackers[1].sensor))
                selection = 1;

        currentSensor = wallTrackers[selection].sensor;
        currentWall = wallTrackers[selection].wall;

        // Query the hardware and return the result:
        Stats.startTimer("io::getDistance");
        double measurement = currentSensor.hardware.getDistance(DistanceUnit.INCH);
        Stats.endTimer("io::getDistance");

        Stats.addData("getDistance", measurement); // @@@ Keep?

        return new Result(currentSensor, currentWall, LATENCY, measurement);
    }

    // Get the most recently used wall segment:
    public Segment getCurrentSegment() {
        return currentWall;
    }

    // Given a pose and a measured distance, calculate a corrected pose that respects that distance:
    static public Pose2d localize(
            Pose2d pose, // Current pose
            SensorState sensor, // Distance sensor descriptor
            Segment wallSegment, // Wall segment
            double measuredDistance, // Distance measured by the sensor
            Poser.HistoryRecord history) // Null if correcting April-tag pose, must be non-null otherwise
    {
        boolean valid = false; // Assume failure, we'll convert to success later

        // Sensor offset from robot center in field coordinates:
        double poseHeading = pose.heading.log();
        Point sensorFieldOffset = sensor.descriptor.offset.rotate(poseHeading);
        double sensorFieldAngle = sensor.descriptor.theta + poseHeading;

        // Create the rays representing the edges of the field-of-view:
        Point sensorPoint = new Point(pose.position).add(sensorFieldOffset);
        Point sensorDirection1 = new Point(Math.cos(sensorFieldAngle - HALF_FOV),
                                           Math.sin(sensorFieldAngle - HALF_FOV));
        Point sensorDirection2 = new Point(Math.cos(sensorFieldAngle + HALF_FOV),
                                           Math.sin(sensorFieldAngle + HALF_FOV));
        Ray sensorRay1 = new Ray(sensorPoint, sensorDirection1);
        Ray sensorRay2 = new Ray(sensorPoint, sensorDirection2);

        // Use the results only if the entire field-of-view is contained within the target
        // segment:
        if ((raySegmentIntersection(sensorRay1, wallSegment) != null) &&
            (raySegmentIntersection(sensorRay2, wallSegment) != null)) {

            Point sensorDirection = new Point(Math.cos(sensorFieldAngle), Math.sin(sensorFieldAngle));
            Ray sensorRay = new Ray(sensorPoint, sensorDirection);

            Point hitPoint = raySegmentIntersection(sensorRay, wallSegment);
            Globals.assertion(hitPoint != null);

            // Note that this hitVector points from the hit-point to the sensor point:
            Point hitVector = sensorPoint.subtract(hitPoint);
            double hitLength = hitVector.length();

            // Ignore measurements that are much shorter than expected (as can happen when
            // another robot blocks the view) or much farther:
            if (Math.abs(hitLength - measuredDistance) < CORRECTNESS_THRESHOLD) {
                // The raw measured result passes all of our conditions! If we're adding to the
                // history (as opposed to doing an April-tag fixup), filter the result (and add
                // it to the filter history):
                if (history != null) {
                    measuredDistance = history.distance.sensor.filter.filter(
                            history.time, measuredDistance, hitLength);
                }

                // Yay, it looks like a valid result. Push the pose out, or pull it in,
                // accordingly:
                double fixupFactor = measuredDistance / hitLength;
                Point fixedUpSensorPoint = hitPoint.add(hitVector.multiply(fixupFactor));
                Point fixupVector = fixedUpSensorPoint.subtract(sensorPoint);

                pose = new Pose2d(pose.position.plus(fixupVector.vector2d()), pose.heading);
                valid = true;
            }
        }

        // Update telemetry:
        if (history != null) {
            history.distance.valid = valid;
            history.distance.point = new Point(
                    pose.position.x + sensorFieldOffset.x + measuredDistance * Math.cos(sensorFieldAngle),
                    pose.position.y + sensorFieldOffset.y + measuredDistance * Math.sin(sensorFieldAngle));
        }

        return pose;
    }
}

/**
 * Localizer for April Tags.
 */
class AprilTagLocalizer {
    // We arbitrarily decide that a pose has to be within this many inches of the old to be reliable:
    final double FINE_EPSILON = 5.0;
    final double COARSE_EPSILON = 24.0;

    // Arbitrary reliable range for an April Tag pose:
    final double RELIABLE_RANGE = 60.0;

    // Activate a camera only if it's within this many inches of a tag:
    final double MIN_ACTIVATION_DISTANCE = 72;

    private CameraState[] cameras = new CameraState[CAMERA_DESCRIPTORS.length];
    int activeCamera = -1; // Currently active camera, -1 if none is

    ArrayList<PoseVisualization> visualizations = new ArrayList<>(); // Most recent April Tags
    double visualizationsTime; // Time of the last visualizations update

    // Camera state:
    static class CameraState {
        boolean enabled; // Enabled by settings
        CameraDescriptor descriptor; // Describes the associated camera
        AprilTagProcessor aprilTagProcessor;
        VisionPortal visionPortal;

        public CameraState(AprilTagProcessor aprilTagProcessor, VisionPortal visionPortal, CameraDescriptor descriptor) {
            this.aprilTagProcessor = aprilTagProcessor; this.visionPortal = visionPortal; this.descriptor = descriptor;
        }
    }

    // Descriptions of attached cameras:
    static final CameraDescriptor[] CAMERA_DESCRIPTORS = {
            new CameraDescriptor("webcam2", new Point(-5.75, -6),
                    new Point(6.0, -5.75), // @@@ Remove me!
                    Math.PI, new Size(1280, 720),
                    906.940247073, 906.940247073, 670.833056673, 355.34234068,
                    0.190, Math.toRadians(75)),
            new CameraDescriptor("webcam1", new Point(7.0, -0.5),
                    new Point(-0.5, -7.0),  // @@@ Remove me!
                    0, new Size(640, 480),
                    -1, -1, -1, -1, // Use default FTC calibration
                    0.190, Math.toRadians(70.4)),
    };

    // Structure for describing cameras on the robot:
    static class CameraDescriptor {
        String name; // Device name in the robot's configuration
        Point goodOffset; // Offset of the camera on the robot
        Point badOffset; // *BAD* Offset of the camera on the robot
        double theta; // Orientation of the sensor on the robot, in radians
        Size resolution; // Resolution in pixels
        double fx, fy, cx, cy; // Lens intrinsics; fx = -1 to use system default
        double latency; // End-to-end April Tag processing latency, in seconds
        double fov; // Horizontal field of view, in radians

        public CameraDescriptor(String name, Point goodOffset, Point badOffset, double theta, Size resolution, double fx, double fy, double cx, double cy, double latency, double fov) {
            this.name = name; this.goodOffset = goodOffset; this.badOffset = badOffset; this.theta = theta; this.resolution = resolution;
            this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
            this.latency = latency; this.fov = fov;
        }
    }

    final Location[] TAG_LOCATIONS = {
        new Location(1, 62.875, 42.750, Math.toRadians(180), false), // Blue left backdrop, small
        new Location(2, 62.875, 36.625, Math.toRadians(180), false), // Blue middle backdrop, small
        new Location(3, 62.875, 30.625, Math.toRadians(180), false), // Blue right backdrop, small
        new Location(4, 62.875, -30.625, Math.toRadians(180), false), // Red left backdrop, small
        new Location(5, 62.875, -36.750, Math.toRadians(180), false), // Red middle backdrop, small
        new Location(6, 62.875, -42.625, Math.toRadians(180), false), // Red right backdrop, small
        new Location(7, -72, -43.0, 0, true),   // Red audience wall, large
        new Location(8, -72, -37.5, 0, false),  // Red audience wall, small
        new Location(9, -72, 37.5, 0, false),  // Blue audience wall, small
        new Location(10, -72, 43.0, 0, true),   // Blue audience wall, large
    };

    // Structure defining the location of April Tags:
    static class Location {
        int id;
        Point point;
        double theta; // Radians
        boolean large;

        Location(int id, double x, double y, double theta, boolean large) {
            this.id = id; this.point = new Point(x, y); this.theta = theta; this.large = large;
        }
    }

    // Structure for visualizing poses:
    static class PoseVisualization {
        enum Quality { LOW, MEDIUM, HIGH }
        Pose2d pose;
        Quality quality;
        public PoseVisualization(Pose2d pose, Quality quality) {
            this.pose = pose; this.quality = quality;
        }
    }

    // Structure for returning April Tag results from update():
    static class Result {
        Pose2d pose; // If non-null, the resulting pose
        boolean isConfident; // Confidence in this pose
        int cameraIndex; // CAMERA_DESCRIPTORS index
        double latency; // Latency, in seconds

        public Result(Pose2d pose, boolean isConfident, int cameraIndex, double latency) {
            this.pose = pose; this.isConfident = isConfident; this.cameraIndex = cameraIndex; this.latency = latency;
        }
    }

    // Enable or disable the live view for "scrcpy" for all cameras:
    void enableLiveView(boolean enable) {
        for (CameraState camera: cameras) {
            if (enable)
                camera.visionPortal.resumeLiveView();
            else
                camera.visionPortal.stopLiveView();
        }
    }

    // Constructor:
    AprilTagLocalizer(HardwareMap hardwareMap) {
        // For multiple simultaneous cameras, have to call 'makeMultiPortalView()' and then pass
        // each container ID into 'setCameraMonitorViewId()':
        int[] containerIds = VisionPortal.makeMultiPortalView(
                CAMERA_DESCRIPTORS.length, VisionPortal.MultiPortalLayout.HORIZONTAL);

        for (int i = 0; i < CAMERA_DESCRIPTORS.length; i++) {
            cameras[i] = initializeCamera(i, hardwareMap, containerIds[i]);
            final int lambdaIndex = i;
            Settings.registerToggleOption(String.format("Enable %s", cameras[i].descriptor.name),
                    false, enable -> cameras[lambdaIndex].enabled = enable );
        }
        Settings.registerToggleOption("Enable screen preview", false, this::enableLiveView);
    }

    // Create the AprilTagProcessor and VisionPortal objects for the specified camera:
    CameraState initializeCamera(int cameraIndex, HardwareMap hardwareMap, int containerId) {
        CameraDescriptor descriptor = CAMERA_DESCRIPTORS[cameraIndex];

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

        AprilTagProcessor aprilTagProcessor = processorBuilder.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, descriptor.name));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(descriptor.resolution);

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false); // We have a setting to override live-view // @@@

        // Set and enable the processor.
        builder.addProcessor(aprilTagProcessor);

        // Set the container ID for multi-camera support:
        builder.setLiveViewContainerId(containerId);

        // Build the Vision Portal, using the above settings.
        VisionPortal visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);

        // Stop streaming to save processing performance:
        // visionPortal.stopStreaming();

        return new CameraState(aprilTagProcessor, visionPortal, descriptor);
    }

    // Close when shutting down:
    void close() {
        for (CameraState camera: cameras)
            camera.visionPortal.close();
    }

    // Find the tag associated with this detection:
    private Location getTag(AprilTagDetection detection) {
        for (Location t : TAG_LOCATIONS) {
            if (t.id == detection.id)
                return t;
        }
        return null;
    }

    // Compute the robot's field-relative pose from the April-tag-relative pose. See
    // https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html.
    private Pose2d computeRobotPose(AprilTagPoseFtc ftcPose, Location tag, CameraDescriptor camera) {
        Pose2d goodResult;
        Pose2d badResult;
        {
            // Compute the field-space angle from the tag to the camera. The triangle formed
            // between the April Tag's plane, the ray going straight out from the camera,
            // and the ray from the April Tag to the camera has three component
            // angles: 1) bearing, 2) 90 degrees - yaw, 3) angle-to-the-robot:
            double angleToRobot = tag.theta + Math.PI/2 + Math.toRadians(ftcPose.yaw - ftcPose.bearing);

            // Compute the camera's field-space location:
            Point vectorToRobot = new Point(ftcPose.range, 0).rotate(angleToRobot);
            Point cameraPoint = tag.point.add(vectorToRobot);

            // Compute the robot's heading by mirroring the angle to the robot, and adding the
            // camera's angle:
            double robotHeading = Math.PI - tag.theta + Math.toRadians(ftcPose.yaw) + camera.theta;

            // Compute the field-space offset from the origin to the camera:
            Point cameraOffset = camera.goodOffset.rotate(robotHeading);
            Point robotCenter = cameraPoint.subtract(cameraOffset);

            goodResult = new Pose2d(robotCenter.vector2d(), robotHeading);
        }

        double dx = ftcPose.x - camera.badOffset.x;
        double dy = ftcPose.y - camera.badOffset.y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        double gamma = -(Math.atan(dx / dy) + Math.toRadians(ftcPose.yaw) + tag.theta);
        double x = tag.point.x + Math.cos(gamma) * distance;
        double y = tag.point.y + Math.sin(gamma) * distance;

        double theta = Math.toRadians(ftcPose.yaw) + tag.theta + camera.theta;
        badResult = new Pose2d(new Vector2d(x, y), Math.PI - theta);

        // System.out.println(String.format("Bad: %s, Good: %s", badResult.toString(), goodResult.toString()));

        return badResult;
    }

    // Interim data structure used to track poses determined by the AprilTags:
    static class VisionPose {
        Pose2d pose; // Field-relative robot pose
        double residual; // Distance from current pose, in inches
        double range; // Distance from April Tag, in inches
        Location tag; // Tag that determined this pose

        VisionPose(Pose2d pose, double residual, double range, Location tag) {
            this.pose = pose; this.residual = residual; this.range = range; this.tag = tag;
        }
    }

    // Compute the distance between two poses:
    private double distance(Pose2d a, Pose2d b) {
        return Math.hypot(a.position.x - b.position.x, a.position.y - b.position.y);
    }

    // Manage the activation of the cameras:
    private CameraState updateActiveCamera(Pose2d pose, boolean confident) { // Returns null if no active camera
        int resultIndex = -1; // Default to none
        if (!confident) {
            // Always keep the primary camera active when locking into a good pose, assuming that
            // it's active:
            resultIndex = 0;
            if (!cameras[resultIndex].enabled)
                resultIndex = -1;
        } else {
            // Determine the camera with the closest view of a tag:
            double minDistance = Double.MAX_VALUE;
            for (Location location: TAG_LOCATIONS) {
                Vector2d vectorToTag = location.point.vector2d().minus(pose.position);
                double tagAngle = Math.atan2(vectorToTag.y, vectorToTag.x); // Rise over run
                for (int i = 0; i < cameras.length; i++) {
                    CameraState camera = cameras[i];
                    if (camera.enabled) {
                        // Check to see if the tag is in the cone for this camera:
                        double cameraAngle = pose.heading.log() + camera.descriptor.theta;
                        // Widen the FOV to account for imprecise pose estimates:
                        double halfFov = (camera.descriptor.fov / 2) * 1.3;
                        double rightAngle = cameraAngle - halfFov;
                        double leftAngle = cameraAngle + halfFov;
                        double deltaRight = Globals.normalizeAngle(tagAngle - rightAngle);
                        double deltaLeft = Globals.normalizeAngle(leftAngle - tagAngle);
                        if ((deltaRight > 0) && (deltaLeft > 0)) {
                            double distance = Math.hypot(vectorToTag.x, vectorToTag.y);
                            if (distance < minDistance) {
                                resultIndex = i;
                                minDistance = distance;
                            }
                        }
                    }
                }
            }

            // Disable if the closest tag is further than this amount:
            if (minDistance > MIN_ACTIVATION_DISTANCE) {
                resultIndex = -1;
            }
        }

        // Do more work if there's a change in the active camera:
        if (activeCamera != resultIndex) {
            // We can't have more than one portal enabled at a time so disable the portals first:
            // @@@ True?
            for (int i = 0; i < cameras.length; i++) {
                cameras[i].visionPortal.setProcessorEnabled(cameras[i].aprilTagProcessor, false);
                cameras[i].visionPortal.stopLiveView();
            }

            // Then enable the specified portal, if there is one:
            if (resultIndex != -1) {
                cameras[resultIndex].visionPortal.setProcessorEnabled(cameras[resultIndex].aprilTagProcessor, true);
                cameras[resultIndex].visionPortal.resumeLiveView();
            }

            Stats.poseStatus = "";
            Stats.cameraFps = 0;
            Stats.pipelineLatency = 0;
            activeCamera = resultIndex;
        }

        return (activeCamera == -1) ? null : cameras[activeCamera];
    }

    // Update loop for April Tags:
    public Result update(Pose2d currentPose, boolean confident) { // Returns null if no new updates
        ArrayList<VisionPose> visionPoses = new ArrayList<>();
        double minResidual = Float.MAX_VALUE;

        CameraState camera = updateActiveCamera(currentPose, confident);
        if (camera == null)
            return null; // ====>

        Stats.startTimer("io::getFreshDetections");
        List<AprilTagDetection> tagDetections = camera.aprilTagProcessor.getFreshDetections();
        Stats.endTimer("io::getFreshDetections");

        if (tagDetections == null)
            return null; // ====>

        Stats.cameraFps = camera.visionPortal.getFps();
        for (AprilTagDetection detection : tagDetections) {
            Stats.pipelineLatency = (nanoTime() - tagDetections.get(0).frameAcquisitionNanoTime) * 1e-9;
            if (detection.metadata != null) {
                Location tag = getTag(detection);
                if (tag != null) {
                    Pose2d visionPose = computeRobotPose(detection.ftcPose, tag, camera.descriptor);
                    double residual = Math.hypot(
                            currentPose.position.x - visionPose.position.x,
                            currentPose.position.y - visionPose.position.y);

                    minResidual = Math.min(minResidual, residual);
                    visionPoses.add(new VisionPose(visionPose, residual, detection.ftcPose.range, tag));
                }
            }
        }

        // This will be the vision pose that we recommend to update the current pose:
        VisionPose visionPose = null;
        int poseCount = visionPoses.size();

        // We love it when there are 3 accurate poses reasonably when reasonably close to
        // the tag - we always use it to set our current pose, regardless of the state of the
        // old pose, and we trust its heading result:
        boolean isExcellentPose = false;
        if ((poseCount == 3) && (tagDetections.get(0).ftcPose.range < RELIABLE_RANGE)) {
            double maxNeighborDistance = 0;

            // Compute the distance between each pose and its neighbors:
            double[] interDistances = new double[3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    if (i != j) {
                        double interDistance = distance(visionPoses.get(i).pose, visionPoses.get(j).pose);
                        interDistances[i] += interDistance;
                        maxNeighborDistance = Math.max(interDistance, maxNeighborDistance);
                    }
                }
            }

            // If all three pose estimates are reasonably close, choose the one with the
            // shortest distance to the other two:
            if (maxNeighborDistance <= FINE_EPSILON) {
                double min = Math.min(Math.min(interDistances[0], interDistances[1]), interDistances[2]);
                for (int i = 0; i < 3; i++) {
                    if (interDistances[i] == min) {
                        isExcellentPose = true;
                        visionPose = visionPoses.get(i);
                        Stats.poseStatus = String.format("Excellent vision 3-pose, min %.2f, max %.2f", min, maxNeighborDistance);
                    }
                }
            }
        }

        // If visionPose is null, we didn't find a perfect pose, so lower our standards a little:
        if (visionPose == null)  {
            // Set a default status:
            if (poseCount == 0) {
                Stats.poseStatus = "No vision pose found";
            } else if (poseCount == 1) {
                Stats.poseStatus = "One inadequate vision pose";
            } else {
                Stats.poseStatus = String.format("%d inadequate vision poses", poseCount);
            }
            if ((poseCount == 1) && (minResidual < FINE_EPSILON) && (visionPoses.get(0).range < RELIABLE_RANGE)) {
                // TODO: Check the returned size?
                // If only a single tag is in view, adopt its pose only if it's relatively close
                // to our current pose estimate:
                Stats.poseStatus = String.format("Good single vision pose, %.2f", visionPoses.get(0).residual);
                visionPose = visionPoses.get(0);
            } else if ((poseCount > 1) && (minResidual < COARSE_EPSILON)) {
                // If multiple tags are in view, choose the closest one to our current pose
                // (reasoning that it's unlikely that *both* are wildly bogus).
                for (VisionPose candidatePose : visionPoses) {
                    if ((candidatePose.residual == minResidual) && (candidatePose.range < RELIABLE_RANGE)) {
                        visionPose = candidatePose;
                        Stats.poseStatus = String.format("Good one in %d vision pose, %.2f residual", visionPoses.size(), candidatePose.residual);
                    }
                }
            }
        }

        Pose2d resultPose = (visionPose != null) ? visionPose.pose : null;

        // Update visualizations:
        visualizationsTime = Globals.time();
        visualizations = new ArrayList<>();
        for (VisionPose pose: visionPoses) {
            PoseVisualization.Quality quality = PoseVisualization.Quality.LOW;
            if (pose == visionPose) {
                quality = (isExcellentPose) ? PoseVisualization.Quality.HIGH : PoseVisualization.Quality.MEDIUM;
            }
            visualizations.add(new PoseVisualization(pose.pose, quality));
        }

        return new Result(resultPose, isExcellentPose, activeCamera, camera.descriptor.latency);
    }
}

/**
 * Localizer for the Optical Tracking sensor.
 */
class OpticalFlowLocalizer {
    OpticalFlowDescriptor[] OPTICAL_DESCRIPTORS = {
//        new OpticalDescriptor("optical1", 0.003569, Math.toRadians(90.42), new Point(-3.34, -0.59)),
        new OpticalFlowDescriptor("optical2", 0.001035, Math.toRadians(90.78), new Point(-4.47, 2.6)),
    };

    // Structure for describing cameras on the robot:
    static class OpticalFlowDescriptor {
        String name; // Device name in the robot's configuration
        double inchesPerTick;
        double headingAdjustment;
        Point offset;

        public OpticalFlowDescriptor(String name, double inchesPerTick, double headingAdjustment, Point offset) {
            this.name = name; this.inchesPerTick = inchesPerTick; this.headingAdjustment = headingAdjustment; this.offset = offset;
        }
    }

    OpticalTrackingPaa5100 device; // SDK device object
    double previousYaw; // Yaw from the previous call to update()
    Pose2d sensorPose; // Pose for the sensor where it's on the robot (not pose for the robot center)
    Pose2d inferiorSensorPose; // Inferior pose as determined by the sensor
    Pose2d robotPose; // Pose for the robot from the last iteration
    boolean visualizeInferiorPose; // Setting for whether to show the inferior pose or not
    OpticalFlowDescriptor descriptor; // Describes the parameters of the sensor

    OpticalFlowLocalizer(HardwareMap hardwareMap) {
        descriptor = OPTICAL_DESCRIPTORS[0];
        device = hardwareMap.get(OpticalTrackingPaa5100.class, descriptor.name);
        device.getMotion(); // Zero the movement
        previousYaw = Globals.getYaw();
        setPose(new Pose2d(0, 0, 0));

        Settings.registerToggleOption("Toggle optical inferior pose", false, enable -> visualizeInferiorPose = enable );
    }

    // Set the pose once it's locked in:
    void setPose(Pose2d pose) {
        double poseHeading = pose.heading.log();
        Point fieldSensorOffset = descriptor.offset.rotate(poseHeading);

        sensorPose = new Pose2d(pose.position.minus(fieldSensorOffset.vector2d()), poseHeading);
        inferiorSensorPose = new Pose2d(pose.position.minus(fieldSensorOffset.vector2d()), poseHeading);
        robotPose = pose;
    }

    // Convert the robot-relative change-in-position to field-relative change-in-pose.
    // Apply the conversion via pose exponentials courtesy of Theorem 10.2.1
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf#page=194&zoom=100,57,447:
    static Point deltaFieldPosition(double theta, double deltaX, double deltaY, double deltaTheta) {
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);
        double sinDeltaThetaOverDeltaTheta = 1 - Math.pow(deltaTheta, 2) / 6;
        double cosDeltaThetaMinusOneOverDeltaTheta = -deltaTheta / 2;
        double oneMinusCosDeltaThetaOverDeltaTheta = deltaTheta / 2;
        double deltaXPrime
                = (cosTheta * sinDeltaThetaOverDeltaTheta - sinTheta * oneMinusCosDeltaThetaOverDeltaTheta) * deltaX
                + (cosTheta * cosDeltaThetaMinusOneOverDeltaTheta - sinTheta * sinDeltaThetaOverDeltaTheta) * deltaY;
        double deltaYPrime
                = (sinTheta * sinDeltaThetaOverDeltaTheta + cosTheta * oneMinusCosDeltaThetaOverDeltaTheta) * deltaX
                + (sinTheta * cosDeltaThetaMinusOneOverDeltaTheta + cosTheta * sinDeltaThetaOverDeltaTheta) * deltaY;
        return new Point(deltaXPrime, deltaYPrime);
    }

    static Point inferiorDeltaFieldPosition(double theta, double deltaX, double deltaY, double deltaTheta) {
        return new Point(deltaX, deltaY).rotate(theta + deltaTheta);
    }

    Poser.Twist update() {
        // Do our I/O:
        Stats.startTimer("io::getMotion");
        OpticalTrackingPaa5100.Motion motion = device.getMotion();
        Stats.endTimer("io::getMotion");

        double currentYaw = Globals.getYaw();

        // Query the quality:
        Stats.addData("Optical tracker quality", device.getQuality());

        // Calculate the angles:
        double deltaTheta = Globals.normalizeAngle(currentYaw - previousYaw);
        previousYaw = currentYaw;
        double theta = sensorPose.heading.log();

        Point motionVector
                = new Point(motion.x, motion.y).multiply(descriptor.inchesPerTick).rotate(descriptor.headingAdjustment);
        Point deltaPosition = deltaFieldPosition(theta, motionVector.x, motionVector.y, deltaTheta);

        // Update the pose for the sensor and the robot center, respectively:
        double xPrime = sensorPose.position.x + deltaPosition.x;
        double yPrime = sensorPose.position.y + deltaPosition.y;
        double thetaPrime = theta + deltaTheta;
        Point centerOffset = descriptor.offset.negate().rotate(thetaPrime);

        sensorPose = new Pose2d(xPrime, yPrime, thetaPrime);
        Pose2d newRobotPose = new Pose2d(xPrime - centerOffset.x, yPrime - centerOffset.y, thetaPrime);
        Poser.Twist twist = new Poser.Twist(newRobotPose, robotPose);
        robotPose = newRobotPose;

        // #########################################################################################
        theta = inferiorSensorPose.heading.log();
        deltaPosition = inferiorDeltaFieldPosition(theta, motionVector.x, motionVector.y, deltaTheta);
        xPrime = inferiorSensorPose.position.x + deltaPosition.x;
        yPrime = inferiorSensorPose.position.y + deltaPosition.y;
        thetaPrime = theta + deltaTheta;
        inferiorSensorPose = new Pose2d(xPrime, yPrime, thetaPrime);
        Pose2d inferiorRobotPose = new Pose2d(xPrime - centerOffset.x, yPrime - centerOffset.y, thetaPrime);

        // Draw the poses:
        if (visualizeInferiorPose) {
            Globals.canvas.setStroke("#ff0000"); // Red
            MecanumDrive.drawRobot(Globals.canvas, inferiorRobotPose);
        }

        Globals.canvas.setStroke("#E9967A"); // Olive
        MecanumDrive.drawRobot(Globals.canvas, robotPose);

        return twist;
    }
}

/**
 * Master localizer for doing pose estimates. Incorporates odometry, April Tags and distance
 * sensors.
 */
public class Poser {
    // These public fields are updated after every call to update():
    public Pose2d bestPose; // Best pose, for external use (rate-limited):
    public PoseVelocity2d velocity; // Current velocity, robot-relative not field-relative

    // Final pose estimate from the previous sensor loop iteration (raw, non-rate-limited):
    private Pose2d posteriorPose;

    // Keep the history around for this many seconds:
    private final static double HISTORY_DURATION = 2.0;

    // Clamp limits:
    /** @noinspection FieldCanBeLocal*/
    private final double MAX_POSITION_CHANGE_RATE = 12.0; // 12 inches/s
    private final double MAX_HEADING_CHANGE_RATE = Math.toRadians(27); // 27 degrees/s

    // True if using optical flow for our master predictions, false if using odometry:
    /** @noinspection FieldCanBeLocal*/
    private final boolean USE_OPTICAL_FLOW = true;

    // The historic record:
    private LinkedList<HistoryRecord> history = new LinkedList<>(); // Newest first
    private double originalYaw; // Radians
    double previousTime; // Time of last call to update();

    // Component localizers:
    private Localizer odometryLocalizer;
    private OpticalFlowLocalizer opticalFlowLocalizer;
    private DistanceLocalizer distanceLocalizer;
    private AprilTagLocalizer aprilTagLocalizer;
    private AprilTagFilter aprilTagFilter;

    // True if an initial pose was specified or if April Tags now have locked-on pose tracking:
    boolean isLockedOn;

    // Record structure for the history:
    static class HistoryRecord {
        double time; // Time, in seconds, of the record
        Pose2d posteriorPose; // Result of the *previous* step
        Twist odometryTwist; // Odometry twist
        Twist opticalFlowTwist; // Optical flow twist, if any
        DistanceLocalizer.Result distance; // Distance sensor distance, if any
        DistanceLocalizer.WallTracker[] wallTrackers; // Distance sensor tracking walls
        AprilTagLocalizer.Result aprilTag; // April tag result, if any

        public HistoryRecord(double time, Pose2d posteriorPose, Twist odometryTwist, Twist opticalFlowTwist) {
            this.time = time; this.posteriorPose = posteriorPose; this.odometryTwist = odometryTwist; this.opticalFlowTwist = opticalFlowTwist;
        }
    }

    // Record structure that encodes the robot-relative change in pose:
    static class Twist {
        Point robotTranslation; // Robot-relative translation relative to the old pose
        double rotation; // Amount of rotation

        // Encode the change in pose to be relative to the robot's orientation:
        Twist(Pose2d newPose, Pose2d oldPose) {
            double theta = oldPose.heading.log();
            Vector2d fieldTranslation  = newPose.position.minus(oldPose.position);
            robotTranslation = new Point(fieldTranslation).rotate(-theta);
            rotation = newPose.heading.log() - theta;
        }

        // Apply the delta to the pose:
        Pose2d apply(Pose2d pose) {
            double theta = pose.heading.log();
            Point fieldTranslation = robotTranslation.rotate(theta);
            return new Pose2d(pose.position.plus(fieldTranslation.vector2d()), theta + rotation);
        }
    }

    // Poser constructor:
    @SuppressLint("DefaultLocale")
    public Poser(HardwareMap hardwareMap, MecanumDrive drive, Pose2d initialPose) {
        isLockedOn = false; // Gets lazily set to 'true'
        posteriorPose = (initialPose != null) ? initialPose : new Pose2d(0, 0, 0);
        velocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        bestPose = posteriorPose;
        previousTime = Globals.time();
        drive.poseOwnedByPoser = true; // Let MecanumDrive know we own pose estimation

        odometryLocalizer = drive.localizer;
        distanceLocalizer = new DistanceLocalizer(hardwareMap);
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        if (USE_OPTICAL_FLOW)
            opticalFlowLocalizer = new OpticalFlowLocalizer(hardwareMap);
        aprilTagFilter = new AprilTagFilter(initialPose != null);

        // Add a menu option to reset the IMU yaw:
        Settings.registerActivationOption("", reset -> {
            if (reset)
                originalYaw = Globals.getYaw();
            return String.format("Reset IMU yaw (%.2f°)",
                Math.toDegrees(Globals.getYaw() - originalYaw));
        });
    }

    // Copy a pose:
    static Pose2d copyPose(Pose2d pose) {
        return new Pose2d(
                new Vector2d(pose.position.x, pose.position.y),
                new Rotation2d(pose.heading.real, pose.heading.imag));
    }

    // Combine the various poses in a historical record:
    Pose2d historyRecordPose(HistoryRecord record) {
        Pose2d compositePose = record.posteriorPose;
        if (record.opticalFlowTwist != null)
            compositePose = record.opticalFlowTwist.apply(compositePose);
        else
            compositePose = record.odometryTwist.apply(compositePose);

        if (record.distance != null) {
            compositePose = DistanceLocalizer.localize(
                compositePose, record.distance.sensor, record.distance.wall, record.distance.measurement, record);
        }

        if ((record.aprilTag != null) && (record.aprilTag.pose != null)) {
            Pose2d aprilTagPose = record.aprilTag.pose;
            if (record.wallTrackers != null) {
                for (DistanceLocalizer.WallTracker tracker : record.wallTrackers) {
                    // Only use this tracking wall data if there already was a hit so the
                    // residual buffer is non-empty:
                    if ((tracker.wall != null) && (tracker.sensor.filter.residuals.size() != 0)) {
                        double currentDistance = DistanceLocalizer.poseDistanceToWall(
                            compositePose, tracker.sensor, tracker.wall);

                        aprilTagPose = DistanceLocalizer.localize(
                            aprilTagPose, tracker.sensor, tracker.wall, currentDistance, null);
                    }
                }
            }
            compositePose = aprilTagFilter.filter(compositePose, aprilTagPose, record.aprilTag.isConfident);
        }
        return compositePose;
    }


    // Update an April-tag or a distance record for a particular time in the past and then
    // recompute all the history back to the present:
    private Pose2d reviseHistory(
            double time,
            AprilTagLocalizer.Result newAprilTag,
            DistanceLocalizer.Result newDistance) {

        Globals.assertion(history.size() > 0);

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
            Globals.assertion(nextTime > time);
            Globals.assertion(time <= currentTime);
            if (nextTime - time  < time - currentTime) {
                iteratorRecord = iterator.next();
            }
        }

        // Add the distance record:
        if (newDistance != null) {
            Globals.assertion(iteratorRecord.distance == null);
            if (iteratorRecord.distance == null)
                iteratorRecord.distance = newDistance;
        }

        if (newAprilTag != null) {
            Globals.assertion(iteratorRecord.aprilTag == null);
            if (iteratorRecord.aprilTag == null)
                iteratorRecord.aprilTag = newAprilTag;
        }

        // Now replay all of the history records back so we can recompute the poses up to the
        // current time:
        while (true) {
            Pose2d compositePose = historyRecordPose(iteratorRecord);
            if (!iterator.hasPrevious())
                return compositePose; // ====>

            iteratorRecord = iterator.previous();
            iteratorRecord.posteriorPose = copyPose(compositePose);
        }
    }

    // Draw cross-hairs at the specified point:
    private void drawCrossHairs(Vector2d position) {
        final double radius = 3;
        Globals.canvas.strokeLine(position.x, position.y - radius, position.x, position.y + radius);
        Globals.canvas.strokeLine(position.x - radius, position.y, position.x + radius, position.y);
    }

    // Draw the history visualizations for FTC Dashboard:
    private void visualize(Pose2d robotPose) {
        Canvas c = Globals.canvas;
        HistoryRecord lastDistance = null;

        // Draw the current distance localizer segment:
        Segment segment = distanceLocalizer.getCurrentSegment();
        if (segment != null) {
            c.setStroke("#800080"); // Purple
            c.setAlpha(0.5);
            c.strokeLine(segment.p1.x, segment.p1.y, segment.p2.x, segment.p2.y);
            c.setAlpha(1.0);
        }

        // Draw the active camera's field of view:
        if (aprilTagLocalizer.activeCamera != -1) {
            AprilTagLocalizer.CameraDescriptor descriptor = AprilTagLocalizer.CAMERA_DESCRIPTORS[aprilTagLocalizer.activeCamera];
            double cameraAngle = robotPose.heading.log() + descriptor.theta;
            Point cameraOffset = descriptor.goodOffset.rotate(robotPose.heading.log());
            Point cameraOrigin = new Point(robotPose.position.x + cameraOffset.x, robotPose.position.y + cameraOffset.y);
            double rayLength = 100;
            double halfFov = descriptor.fov / 2.0;

            c.setStrokeWidth(0); // This seems to be the same as width 1, oh well
            c.setAlpha(0.5);
            c.setStroke("#d0d0d0"); // Almost invisible grey
            c.strokeLine(cameraOrigin.x, cameraOrigin.y,
                    cameraOrigin.x + rayLength * Math.cos(cameraAngle + halfFov),
                    cameraOrigin.y + rayLength * Math.sin(cameraAngle + halfFov));
            c.strokeLine(cameraOrigin.x, cameraOrigin.y,
                    cameraOrigin.x + rayLength * Math.cos(cameraAngle - halfFov),
                    cameraOrigin.y + rayLength * Math.sin(cameraAngle - halfFov));
            c.setAlpha(1.0);
        }

        if (history.size() != 0) {
            // Go from oldest to newest:
            ListIterator<HistoryRecord> iterator = history.listIterator(history.size());
            HistoryRecord record = iterator.previous();
            while (true) {
                // Draw the pose trail:
                c.setFill("#3F51B5");
                c.fillRect(record.posteriorPose.position.x - 0.5,
                           record.posteriorPose.position.y - 0.5, 1, 1);

                // Draw the distance sensor results in gray (rejects) and green (accepted):
                if (record.distance != null) {
                    c.setFill((record.distance.valid) ? "#00ff00" : "#d0d0d0");
                    c.fillRect(record.distance.point.x - 0.5, record.distance.point.y - 0.5,
                            1, 1);
                    lastDistance = record;
                }

                // Go to the next newest record:
                if (!iterator.hasPrevious())
                    break;
                record = iterator.previous();
            }
        }

        // Draw current April Tag poses:
        if (Globals.time() - aprilTagLocalizer.visualizationsTime < 0.5) {
            for (AprilTagLocalizer.PoseVisualization visualization : aprilTagLocalizer.visualizations) {
                if (visualization.quality == AprilTagLocalizer.PoseVisualization.Quality.LOW)
                    c.setStroke("#ff0000"); // Red
                else if (visualization.quality == AprilTagLocalizer.PoseVisualization.Quality.MEDIUM)
                    c.setStroke("a0a000"); // Yellow
                else
                    c.setStroke("00ff00"); // Green
                drawCrossHairs(visualization.pose.position);
            }

            // Draw the AprilTag filter residuals along with their bounding circle in black:
            c.setStroke("#000000");
            CircleFitter.Circle circle = aprilTagFilter.boundingCircle;
            if (circle.r != 0)
                c.strokeCircle(robotPose.position.x + circle.x, robotPose.position.y + circle.y, circle.r);
            c.setStroke("#808080");
            for (AprilTagFilter.Storage<Point> point : aprilTagFilter.positionResiduals) {
                c.fillRect(robotPose.position.x + point.residual.x - 0.5,
                        robotPose.position.y + point.residual.y - 0.5, 1, 1);
            }
        }

        // Draw the last distance measurement if it's new enough:
        if ((lastDistance != null) && (lastDistance.distance.valid) &&
                (Globals.time() - lastDistance.time < DistanceLocalizer.READ_INTERVAL)) {
            c.setStroke("#a0a0a0"); // Grey
            c.setAlpha(0.5);

            double poseHeading = robotPose.heading.log();
            Point offset = lastDistance.distance.sensor.descriptor.offset.rotate(poseHeading);
            Point origin = new Point(lastDistance.posteriorPose.position).add(offset);
            c.strokeLine(origin.x, origin.y, lastDistance.distance.point.x, lastDistance.distance.point.y);
            c.setAlpha(1.0);
        }

        // Finally, draw our current pose, drawing a rectangle instead of using drawRobot().
        // Don't use Canvas's 'setRotation()' and 'setTranslation()' because there is no
        // apparent clean way to robustly reset them.
        double halfWidth = 18.5 / 2;
        double halfLength = 17.5 / 2;
        double theta = robotPose.heading.log();
        Point offset = new Point(robotPose.position);
        c.setStroke("#3F51B5");
        Point[] corners = {
                new Point(-halfWidth, halfLength).rotate(theta).add(offset),
                new Point(halfWidth, halfLength).rotate(theta).add(offset),
                new Point(halfWidth, -halfLength).rotate(theta).add(offset),
                new Point(-halfWidth, -halfLength).rotate(theta).add(offset),
                new Point(-halfWidth, halfLength).rotate(theta).add(offset)
        };

        // Who thought it was a good idea for Polyline to take separate arrays for x and for y?
        double[] xArray = new double[corners.length];
        double[] yArray = new double[corners.length];
        for (int i = 0; i < corners.length; i++) {
            xArray[i] = corners[i].x;
            yArray[i] = corners[i].y;
        }
        c.strokePolyline(xArray, yArray);

        // Draw the direction segment from the middle of the robot to the middle of the front:
        Point middleFront = new Point(halfWidth, 0).rotate(theta).add(offset);
        c.strokeLine(robotPose.position.x, robotPose.position.y, middleFront.x, middleFront.y);
    }

    // Update the pose estimate:
    public void update() {
        // Track the current time and the delta-t from the previous update() call:
        double time = Globals.time();
        double dt = time - previousTime;
        previousTime = time;

        // Check to see if we're no locked on and if so, broadcast to all interested
        // pose estimators:
        if ((!isLockedOn) && aprilTagFilter.isConfidentFilter()) {
            isLockedOn = true;
            if (opticalFlowLocalizer != null)
                opticalFlowLocalizer.setPose(posteriorPose);
        }

        // Update odometry. Note that MecanumDrive's pose gets updated by 'doActionsWork':
        Twist2dDual<Time> dualTwist = odometryLocalizer.update();
        velocity = dualTwist.velocity().value();
        Twist odometryTwist = new Twist(posteriorPose.plus(dualTwist.value()), posteriorPose);

        // Update the optical flow localizer:
        Twist opticalFlowTwist = (opticalFlowLocalizer != null) ? opticalFlowLocalizer.update() : null;

        // Create a tracking record and update the master pose accordingly. This may
        // get recomputed via 'reviseHistory()' calls below:
        HistoryRecord record = new HistoryRecord(time, posteriorPose, odometryTwist, opticalFlowTwist);
        history.addFirst(record); // Newest first
        posteriorPose = historyRecordPose(record);

        // Incrementally update the best pose the same way:
        bestPose = historyRecordPose(new HistoryRecord(time, bestPose, odometryTwist, opticalFlowTwist));

        // Delete expired history records:
        while (time - history.getLast().time > HISTORY_DURATION) {
            history.removeLast(); // Oldest last
        }

        // Process April Tags:
        AprilTagLocalizer.Result aprilTag = aprilTagLocalizer.update(posteriorPose, aprilTagFilter.isConfidentFilter());
        if (aprilTag != null) {
            posteriorPose = reviseHistory(time - aprilTag.latency, aprilTag, null);
        }

        // Process the distance sensor. Only act on it if the current pose estimate is
        // reliable enough, otherwise chaos will ensue:
        DistanceLocalizer.Result distance = distanceLocalizer.update(posteriorPose, record);
        if ((aprilTagFilter.isConfidentFilter()) && (distance != null)) {
            posteriorPose = reviseHistory(time - distance.latency, null, distance);
        }

        if (distanceLocalizer.currentSensor != null)
            Stats.addData("distanceFilter size", distanceLocalizer.currentSensor.filter.residuals.size());
        Stats.addData("aprilTagFilter position size", aprilTagFilter.positionResiduals.size());
        Stats.addData("aprilTagFilter heading size", aprilTagFilter.headingResiduals.size()); // @@@

        // Once we're locked in, clamp the rate of any changes of 'posteriorPose' to become
        // 'bestPose' that all clients use:
        if (!isLockedOn) {
            bestPose = posteriorPose;
        } else {
            Vector2d positionCorrection = posteriorPose.position.minus(bestPose.position);
            double headingCorrection = Globals.normalizeAngle(posteriorPose.heading.log() - bestPose.heading.log());

            double correctionMagnitude = new Point(positionCorrection).length();
            if (correctionMagnitude > dt * MAX_POSITION_CHANGE_RATE) {
                double clampFactor = dt * MAX_POSITION_CHANGE_RATE / correctionMagnitude;
                positionCorrection = positionCorrection.times(clampFactor);
            }
            if (Math.abs(headingCorrection) > dt * MAX_HEADING_CHANGE_RATE) {
                headingCorrection = dt * Math.signum(headingCorrection) * MAX_HEADING_CHANGE_RATE;
            }
            bestPose = new Pose2d(bestPose.position.plus(positionCorrection),
                                  bestPose.heading.log() + headingCorrection);
        }

        // Output telemetry and visualizations:
        visualize(bestPose);
        aprilTagFilter.telemetry();

        Stats.imuYaw = Globals.getYaw() - originalYaw;
        Stats.yawCorrection = posteriorPose.heading.log() - Stats.imuYaw;
    }

    // Close when done running our OpMode:
    void close() {
        aprilTagLocalizer.close();
    }

    // Return true when sufficiently calibrated:
    boolean isLockedOn() {
        return isLockedOn;
    }
}
