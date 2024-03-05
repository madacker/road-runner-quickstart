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
import com.qualcomm.robotcore.hardware.IMU;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.opticaltracking.OpticalTrackingPaa5100;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
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
    static final double WINDOW_DURATION = 0.500; // Seconds
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
            residual.residual = distanceCorrection - residual.residual;
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
    static final double HEADING_WINDOW_DURATION = 20.0; // Seconds
    static final double POSITION_WINDOW_DURATION = 0.500; // Seconds
    static final double WATCHDOG_WINDOW_FRACTION = 0.5; // Fraction of the window duration
    static final int CONFIDENCE_THRESHOLD_COUNT = 10; // Need 10 excellent readings before being confident
    static final double MAX_POSITION_CHANGE_RATE = 18; // Inches per second
    static final double MAX_HEADING_CHANGE_RATE = Math.toRadians(1); // Degrees per second

    private double confidentAprilTagCount = 0; // Active count of trusted headings so far
    private boolean isConfident; // True if we're now confident in the results coming out

    // The residuals are maintained in newest-to-oldest order:
    private LinkedList<Storage<Double>> headingResiduals = new LinkedList<>();
    private LinkedList<Storage<Vector2d>> positionResiduals = new LinkedList<>();

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
    AprilTagFilter(boolean isConfident) {
        this.isConfident = isConfident;
    }

    // Filter the prediction and measurements and return a posterior:
    @NonNull Pose2d filter(
            double time, // Globals.time() for when the filtering should be done
            @NonNull Pose2d odometryPose, // Odometry measured pose
            @Nullable Pose2d aprilTagPose, // April-tag
            boolean confidentAprilTag) { // True if confident in the April-tag

        // We might have gone back in time so remove any newer results:
        while ((positionResiduals.size() > 0) && (positionResiduals.getFirst().time >= time))
            positionResiduals.removeFirst();
        while ((headingResiduals.size() > 0) && (headingResiduals.getFirst().time >= time))
            headingResiduals.removeFirst();

        // If the newest residuals are too stale (meany it's been a long time since we got a
        // new one), toss them all. We don't want to age them out one at a time because that
        // simply biases everything to the most recent one:
        if ((positionResiduals.size() > 0) &&
                (time - positionResiduals.getFirst().time > POSITION_WINDOW_DURATION * WATCHDOG_WINDOW_FRACTION)) {
            positionResiduals.clear();
        }
        if ((headingResiduals.size() > 0) &&
                (time - headingResiduals.getFirst().time > HEADING_WINDOW_DURATION * WATCHDOG_WINDOW_FRACTION)) {
            headingResiduals.clear();
        }

        // Remove all residuals that are too old:
        while ((positionResiduals.size() > 0) && (time - positionResiduals.getLast().time > POSITION_WINDOW_DURATION))
            positionResiduals.removeLast(); // Newest to oldest
        while ((headingResiduals.size() > 0) && (time - headingResiduals.getLast().time > HEADING_WINDOW_DURATION))
            headingResiduals.removeLast(); // Newest to oldest

        // Add April Tag and distance sensor results to the position history:
        if (aprilTagPose != null) {
            Vector2d positionResidual = aprilTagPose.position.minus(odometryPose.position);
            positionResiduals.addFirst(new Storage<>(positionResidual)); // Newest to oldest
        }

        // Add confident April Tags to the headings history:
        if (confidentAprilTag) {
            assert(aprilTagPose != null);
            double headingResidual = Globals.normalizeAngle(aprilTagPose.heading.log() - odometryPose.heading.log());
            headingResiduals.addFirst(new Storage<>(headingResidual)); // Newest to oldest

            confidentAprilTagCount++;
            if (confidentAprilTagCount > CONFIDENCE_THRESHOLD_COUNT)
                isConfident = true;
        }

        // Compute the averages of all the residuals:
        Vector2d aggregatePositionResidual = new Vector2d(0, 0);
        double aggregateHeaderResidual = 0;

        for (Storage<Vector2d> position: positionResiduals) {
            aggregatePositionResidual = aggregatePositionResidual.plus(position.residual);
        }
        for (Storage<Double> heading: headingResiduals) {
            aggregateHeaderResidual = aggregateHeaderResidual + heading.residual;
        }

        if (positionResiduals.size() != 0)
            aggregatePositionResidual = aggregatePositionResidual.div(positionResiduals.size());
        if (headingResiduals.size() != 0)
            aggregateHeaderResidual = aggregateHeaderResidual / headingResiduals.size();

        // Compute how much to correct the current position and heading:
        Vector2d positionCorrection = new Vector2d(aggregatePositionResidual.x, aggregatePositionResidual.y);
        double headingCorrection = aggregateHeaderResidual;


        // Negatively offset everything in the history to account for the offset we're adding:
        for (Storage<Vector2d> position: positionResiduals) {
            position.residual = positionCorrection.minus(position.residual);
        }
        for (Storage<Double> heading: headingResiduals) {
            heading.residual = headingCorrection - heading.residual;
        }

        // Correct the current pose to create the new pose:
        return new Pose2d(
                odometryPose.position.plus(positionCorrection),
                odometryPose.heading.log() + headingCorrection);
    }

    // True if we have high confidence in the results:
    boolean isConfident() {
        return isConfident;
    }

    // Output telemetry:
    void telemetry() {
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
        new DistanceDescriptor("distance", new Point(-3, -2), Math.PI)
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
                    true, enable -> state.enabled = enable );
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

    // Calculate the distance from the sensor to the specified wall:
    static public double distanceToWall(Pose2d pose, SensorState sensor, Segment wall) {
        // Sensor offset from robot center in field coordinates:
        double sensorAngle = sensor.descriptor.theta + pose.heading.log();
        Point sensorOffset = sensor.descriptor.offset.rotate(sensorAngle);

        // Ray representing the sensor's position and orientation on the field:
        Point sensorPoint = new Point(pose.position.x, pose.position.y).add(sensorOffset);
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
                double sensorAngle = sensor.descriptor.theta + pose.heading.log();
                Point sensorOffset = sensor.descriptor.offset.rotate(sensorAngle);

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
            Poser.HistoryRecord history) // For recording telemetry data, can be null
    {
        boolean valid = false; // Assume failure, we'll convert to success later

        // Sensor offset from robot center in field coordinates:
        double sensorFieldAngle = sensor.descriptor.theta + pose.heading.log();
        Point sensorFieldOffset = sensor.descriptor.offset.rotate(sensorFieldAngle);

        // Create the rays representing the edges of the field-of-view:
        Point sensorPoint = new Point(pose.position.x, pose.position.y).add(sensorFieldOffset);
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
            assert(hitPoint != null);
            Point hitVector = new Point(pose.position.x, pose.position.y).subtract(hitPoint);
            double expectedDistance = hitVector.length();

            // Ignore measurements that are much shorter than expected (as can happen when
            // another robot blocks the view) or much farther:
            if (Math.abs(expectedDistance - measuredDistance) < CORRECTNESS_THRESHOLD) {
                // Yay, it looks like a valid result. Push the pose out, or pull it in,
                // accordingly:
                double fixupFactor = measuredDistance / expectedDistance;
                Point fixupVector = new Point(hitVector.x * fixupFactor, hitVector.y * fixupFactor);
                Point newPosition = hitPoint.add(fixupVector);

                pose = new Pose2d(new Vector2d(newPosition.x, newPosition.y), pose.heading);
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
            new CameraDescriptor("webcam2", new Point(6.0, -5.75), Math.PI,
                    new Size(1280, 720),
                    906.940247073, 906.940247073, 670.833056673, 355.34234068,
                    0.19, Math.toRadians(75))
    };

    // Structure for describing cameras on the robot:
    static class CameraDescriptor {
        String name; // Device name in the robot's configuration
        Point offset; // Offset from camera *to* the center (opposite of what you'd expect!), inches
        double theta; // Orientation of the sensor on the robot, in radians
        Size resolution; // Resolution in pixels
        double fx, fy, cx, cy; // Lens intrinsics; fx = -1 to use system default
        double latency; // End-to-end April Tag processing latency, in seconds
        double fov; // Horizontal field of view, in radians

        public CameraDescriptor(String name, Point offset, double theta, Size resolution, double fx, double fy, double cx, double cy, double latency, double fov) {
            this.name = name; this.offset = offset; this.theta = theta; this.resolution = resolution;
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
        double x;
        double y;
        double theta; // Degrees
        boolean large;

        Location(int id, double x, double y, double theta, boolean large) {
            this.id = id; this.x = x; this.y = y; this.theta = theta; this.large = large;
        }
    }

    // Structure for returning April Tag results from update():
    static class Result {
        Pose2d pose; // If non-null, the resulting pose
        boolean isConfident; // Confidence in this pose
        int cameraIndex; // CAMERA_DESCRIPTORS index
        double latency; // Latency, in seconds
        ArrayList<Pose2d> rejectList; // Rejected April Tag poses for telemetry purposes

        public Result(Pose2d pose, boolean isConfident, int cameraIndex, double latency, ArrayList<Pose2d> rejectList) {
            this.pose = pose; this.isConfident = isConfident; this.cameraIndex = cameraIndex; this.latency = latency; this.rejectList = rejectList;
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
        for (int i = 0; i < CAMERA_DESCRIPTORS.length; i++) {
            cameras[i] = initializeCamera(i, hardwareMap);
            final int lambaIndex = i;
            Settings.registerToggleOption(String.format("Enable %s", cameras[i].descriptor.name),
                    true, enable -> cameras[lambaIndex].enabled = enable );
        }
        Settings.registerToggleOption("Enable screen preview", false,
                enable -> enableLiveView(enable));
    }

    // Create the AprilTagProcessor and VisionPortal objects for the specified camera:
    CameraState initializeCamera(int cameraIndex, HardwareMap hardwareMap) {
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
        // builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false); // We have a setting to override live-view

        // Set and enable the processor.
        builder.addProcessor(aprilTagProcessor);

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

    // Compute the robot's field-relative pose from the April-tag-relative pose.
    private Pose2d computeRobotPose(AprilTagDetection detection, Location tag, CameraDescriptor descriptor) {
        double dx = detection.ftcPose.x - descriptor.offset.x;
        double dy = detection.ftcPose.y - descriptor.offset.y;
        double distance = Math.sqrt(dx * dx + dy * dy);

        double gamma = -(Math.atan(dx / dy) + Math.toRadians(detection.ftcPose.yaw) + tag.theta);
        double x = tag.x + Math.cos(gamma) * distance;
        double y = tag.y + Math.sin(gamma) * distance;

        double theta = Math.toRadians(detection.ftcPose.yaw) + tag.theta + descriptor.theta;
        return new Pose2d(new Vector2d(x, y), Math.PI - theta);
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
                Vector2d vectorToTag = new Vector2d(
                        location.x - pose.position.x,
                        location.y - pose.position.y);
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
            for (int i = 0; i < cameras.length; i++) {
                boolean enable = (i == resultIndex);
                cameras[i].visionPortal.setProcessorEnabled(cameras[i].aprilTagProcessor, enable);
            }

            Stats.poseStatus = "";
            Stats.cameraFps = 0;
            Stats.pipelineLatency = 0;
            activeCamera = resultIndex;
        }

        return (activeCamera == -1) ? null : cameras[activeCamera];
    }

    // Update loop for April Tags:
    @SuppressLint("DefaultLocale")
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
                    Pose2d visionPose = computeRobotPose(detection, tag, camera.descriptor);
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
        ArrayList<Pose2d> rejectList = new ArrayList<>();
        for (VisionPose rejectPose : visionPoses) {
            if (rejectPose != visionPose) {
                rejectList.add(rejectPose.pose);
            }
        }

        return new Result(resultPose, isExcellentPose, activeCamera, camera.descriptor.latency, rejectList);
    }
}

/**
 * Localizer for the Optical Tracking sensor.
 */
class OpticalLocalizer {
    OpticalDescriptor[] OPTICAL_DESCRIPTORS = {
        new OpticalDescriptor("optical1", 0.003569, Math.toRadians(90.42), new Point(-3.34, -0.59)),
        new OpticalDescriptor("optical2", 0.001035, Math.toRadians(90.78), new Point(-4.47, 2.6)),
    };

    // Structure for describing cameras on the robot:
    static class OpticalDescriptor {
        String name; // Device name in the robot's configuration
        double inchesPerTick;
        double headingAdjustment;
        Point offset;

        public OpticalDescriptor(String name, double inchesPerTick, double headingAdjustment, Point offset) {
            this.name = name; this.inchesPerTick = inchesPerTick; this.headingAdjustment = headingAdjustment; this.offset = offset;
        }
    }

    OpticalTrackingPaa5100 device;
    IMU imu;
    double previousYaw;
    Pose2d sensorPose;

    OpticalLocalizer(HardwareMap hardwareMap, IMU imu) {
        this.imu = imu;

        OpticalDescriptor descriptor = OPTICAL_DESCRIPTORS[0];

        device = hardwareMap.get(OpticalTrackingPaa5100.class, descriptor.name);
        device.getMotion(); // Zero the movement
        previousYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        sensorPose = new Pose2d(-descriptor.offset.x, -descriptor.offset.y, 0);
    }

    // Convert the robot-relative change-in-pose to field-relative change-in-position.
    // Apply the conversion via Forward Euler integration courtesy of Theorem 10.2.1
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

    Pose2d update() {
        // Do our I/O:
        Stats.startTimer("io::getMotion");
        OpticalTrackingPaa5100.Motion motion = device.getMotion();
        Stats.endTimer("io::getMotion");

        Stats.startTimer("io::getImu2");
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Stats.endTimer("io::getImu2");

        // Calculate the angles:
        double deltaTheta = Globals.normalizeAngle(currentYaw - previousYaw);
        previousYaw = currentYaw;
        double theta = sensorPose.heading.log();

        OpticalDescriptor descriptor = OPTICAL_DESCRIPTORS[0];
        Point motionVector
                = new Point(motion.x, motion.y).scale(descriptor.inchesPerTick).rotate(descriptor.headingAdjustment);
        Point deltaPosition = deltaFieldPosition(theta, motionVector.x, motionVector.y, deltaTheta);

        // Update the pose for the sensor and the robot center, respectively:
        double xPrime = sensorPose.position.x + deltaPosition.x;
        double yPrime = sensorPose.position.y + deltaPosition.y;
        double thetaPrime = theta + deltaTheta;
        Point centerOffset = descriptor.offset.negate().rotate(thetaPrime);

        sensorPose = new Pose2d(xPrime, yPrime, thetaPrime);
        return new Pose2d(xPrime - centerOffset.x, yPrime - centerOffset.y, thetaPrime);
    }
}

/**
 * Master localizer for doing pose estimates. Incorporates odometry, April Tags and distance
 * sensors.
 */
public class Poser {
    // These public fields are updated after every call to update():
    public Pose2d pose; // Current pose
    public PoseVelocity2d velocity; // Current velocity, robot-relative not field-relative

    // Keep the history around for this many seconds:
    private final static double HISTORY_DURATION = 2.0;

    // The historic record:
    private LinkedList<HistoryRecord> history = new LinkedList<>(); // Newest first
    private double originalYaw; // Radians

    // Component localizers:
    private IMU imu;
    private Localizer odometryLocalizer;
    private DistanceLocalizer distanceLocalizer;
    private AprilTagLocalizer aprilTagLocalizer;
    private OpticalLocalizer opticalLocalizer;
    private AprilTagFilter aprilTagFilter;

    // Record structure for the history:
    static class HistoryRecord {
        double time; // Time, in seconds, of the record
        Pose2d posteriorPose; // Result of the *previous* step
        Twist2d twist; // Odometry twist
        DistanceLocalizer.Result distance; // Distance sensor distance, if any
        DistanceLocalizer.WallTracker[] wallTrackers; // Distance sensor tracking walls
        AprilTagLocalizer.Result aprilTag; // April tag result, if any

        public HistoryRecord(double time, Pose2d posteriorPose, Twist2d twist) {
            this.time = time; this.posteriorPose = posteriorPose; this.twist = twist;
        }
    }

    // Constructor:
    @SuppressLint("DefaultLocale")
    public Poser(HardwareMap hardwareMap, MecanumDrive drive, Pose2d initialPose) {
        pose = (initialPose != null) ? initialPose : new Pose2d(0, 0, 0);
        velocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

        imu = drive.imu;
        odometryLocalizer = drive.localizer;
        distanceLocalizer = new DistanceLocalizer(hardwareMap);
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        opticalLocalizer = new OpticalLocalizer(hardwareMap, imu);

        aprilTagFilter = new AprilTagFilter(initialPose != null);

        // Add a menu option to reset the IMU yaw:
        Settings.registerActivationOption("", reset -> {
            if (reset)
                originalYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            return String.format("Reset IMU yaw (%.2f°)",
                Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - originalYaw));
        });
    }

    // Copy a pose:
    static Pose2d copyPose(Pose2d pose) {
        return new Pose2d(
                new Vector2d(pose.position.x, pose.position.y),
                new Rotation2d(pose.heading.real, pose.heading.imag));
    }

    // Combine the various poses in a historical record:
    Pose2d filterHistoryRecord(HistoryRecord record) {
        Pose2d pose = record.posteriorPose.plus(record.twist);

        if (record.distance != null) {
            double estimatedDistance = DistanceLocalizer.distanceToWall(
                    pose, record.distance.sensor, record.distance.wall);
            if (estimatedDistance < Double.MAX_VALUE) {
                double filteredDistance = record.distance.sensor.filter.filter(
                        record.time, record.distance.measurement, estimatedDistance);
                pose = DistanceLocalizer.localize(
                        pose, record.distance.sensor, record.distance.wall, filteredDistance, record);
            }
        }

        if (record.aprilTag != null) {
            Pose2d aprilTagPose = record.aprilTag.pose;
            if (record.wallTrackers != null) {
                for (DistanceLocalizer.WallTracker tracker : record.wallTrackers) {
                    // Only use this tracking wall data if there already was a hit so the
                    // residual buffer is non-empty:
                    if ((tracker.wall != null) && (tracker.sensor.filter.residuals.size() != 0)) {
                        double currentDistance = DistanceLocalizer.distanceToWall(
                                pose, tracker.sensor, tracker.wall);
                        aprilTagPose = DistanceLocalizer.localize(
                                aprilTagPose, tracker.sensor, tracker.wall, currentDistance, null);
                    }
                }
            }
            pose = aprilTagFilter.filter(record.time, pose, aprilTagPose, record.aprilTag.isConfident);
        }
        return pose;
    }

    // Update an April-tag or a distance record for a particular time in the past and then
    // recompute all the history back to the present:
    private void reviseHistory(
            double time,
            AprilTagLocalizer.Result newAprilTag,
            DistanceLocalizer.Result newDistance) {

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
            assert(iteratorRecord.distance == null);
            if (iteratorRecord.distance == null)
                iteratorRecord.distance = newDistance;
        }

        if (newAprilTag != null) {
            assert(iteratorRecord.aprilTag == null);
            if (iteratorRecord.aprilTag == null)
                iteratorRecord.aprilTag = newAprilTag;
        }

        // Now replay all of the history records back so we can recompute the poses up to the
        // current time:
        while (true) {
            pose = filterHistoryRecord(iteratorRecord);
            if (!iterator.hasPrevious())
                break; // ====>

            iteratorRecord = iterator.previous();
            iteratorRecord.posteriorPose = copyPose(pose);
        }

    }

    // Draw the history visualizations for FTC Dashboard:
    private void visualize(Pose2d opticalPose) {
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
            double cameraAngle = pose.heading.log() + descriptor.theta;
            Point cameraOffset = descriptor.offset.rotate(pose.heading.log() - Math.PI / 2); // Account for my (x, y) flip
            Point cameraOrigin = new Point(pose.position.x + cameraOffset.x, pose.position.y + cameraOffset.y);
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

                // Draw April Tag poses, if present:
                if (record.aprilTag != null) {
                    // Draw any distant rejected poses in red:
                    for (Pose2d rejectPose : record.aprilTag.rejectList) {
                        double distance = Float.MAX_VALUE;
                        if (record.aprilTag.pose != null) {
                            Vector2d difference = rejectPose.position.minus(record.aprilTag.pose.position);
                            distance = Math.hypot(difference.x, difference.y);
                        }
                        if (distance > 6) {
                            c.setStroke("#ff0000");
                            MecanumDrive.drawRobot(Globals.canvas, rejectPose, 3);
                        }
                    }

                    if (record.aprilTag.pose != null) {
                        // Draw confident April Tags in green, not so confident in yellow:
                        c.setStroke(record.aprilTag.isConfident ? "#00ff00" : "#a0a000");
                        MecanumDrive.drawRobot(Globals.canvas, record.aprilTag.pose, 3);
                    }
                }

                // Go to the next newest record:
                if (!iterator.hasPrevious())
                    break;
                record = iterator.previous();
            }
        }

        // Draw the last distance measurement if it's new enough:
        if ((lastDistance != null) && (lastDistance.distance.valid) &&
                (Globals.time() - lastDistance.time < DistanceLocalizer.READ_INTERVAL)) {
            c.setStroke("#a0a0a0"); // Grey
            DistanceLocalizer.DistanceDescriptor descriptor
                    = lastDistance.distance.sensor.descriptor;
            Point offset = new Point(descriptor.offset.x, descriptor.offset.y)
                    .rotate(lastDistance.posteriorPose.heading.log() - Math.PI / 2);
            Point origin = new Point(
                    lastDistance.posteriorPose.position.x + offset.x,
                    lastDistance.posteriorPose.position.y + offset.y);
            c.strokeLine(origin.x, origin.y, lastDistance.distance.point.x, lastDistance.distance.point.y);
        }

        // Draw the optical pose:
        c.setStroke("#E9967A"); // Olive
        MecanumDrive.drawRobot(Globals.canvas, opticalPose);

        // Finally, draw our current pose, drawing a rectangle instead of using drawRobot():
        double halfWidth = 18.5 / 2;
        double halfLength = 17.5 / 2;
        c.setStroke("#3F51B5");
        c.setRotation(pose.heading.log());
        c.setTranslation(pose.position.x, pose.position.y);
        c.strokePolyline(new double[] { -halfWidth, halfWidth, halfWidth, -halfWidth, -halfWidth },
                         new double[] { halfLength, halfLength, -halfLength, -halfLength, halfLength });
        c.strokeLine(0, 0, halfWidth, 0);
    }

    // Update the pose estimate:
    public void update() {
        double time = Globals.time();

        Stats.startTimer("odom");
        Twist2dDual<Time> dualTwist = odometryLocalizer.update();
        Stats.endTimer("odom");

        velocity = dualTwist.velocity().value();

        // Create a tracking record:
        HistoryRecord record = new HistoryRecord(time, pose, dualTwist.value());
        history.addFirst(record);

        // Update the pose accordingly. This may get recomputed via 'changeHistory' below:
        pose = filterHistoryRecord(record);

        // Delete old history records:
        while (time - history.getLast().time > HISTORY_DURATION) {
            history.removeLast();
        }

        // Process April Tags:
        Stats.startTimer("aprilTag");
        AprilTagLocalizer.Result aprilTag = aprilTagLocalizer.update(pose, aprilTagFilter.isConfident());
        if (aprilTag != null) {
            reviseHistory(time - aprilTag.latency, aprilTag, null);
        }
        Stats.endTimer("aprilTag");

        // Process the distance sensor. Only act on it if the current pose estimate is
        // reliable enough, otherwise chaos will ensue:
        DistanceLocalizer.Result distance = distanceLocalizer.update(pose, record);
        if ((aprilTagFilter.isConfident()) && (distance != null)) {
            reviseHistory(time - distance.latency, null, distance);
        }

        // Update the optical localizer:
        Pose2d opticalPose = opticalLocalizer.update();

        // @@@ Think about clamping rate change

//        // Track the current time and the delta-t from the previous filter operation:
//        double dt = time - previousTime;
//        previousTime = time;
//
//        // Once we're locked in, clamp the rate of any changes:
//        if (isConfident) {
//            double distance = Math.hypot(aggregatePositionResidual.x, aggregatePositionResidual.y);
//            if (distance > dt * MAX_POSITION_CHANGE_RATE) {
//                double clampFactor = dt * MAX_POSITION_CHANGE_RATE / distance;
//                positionCorrection = positionCorrection.times(clampFactor);
//            }
//            if (Math.abs(headingCorrection) > dt * MAX_HEADING_CHANGE_RATE) {
//                headingCorrection = dt * Math.signum(headingCorrection) * MAX_HEADING_CHANGE_RATE;
//            }
//        }

        // Output telemetry and visualizations:
        visualize(opticalPose);
        aprilTagFilter.telemetry();

        Stats.imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - originalYaw;
        Stats.yawCorrection = pose.heading.log() - Stats.imuYaw;
    }

    // Close when done running our OpMode:
    void close() {
        aprilTagLocalizer.close();
    }

    // Return true when sufficiently calibrated:
    boolean isConfident() {
        return aprilTagFilter.isConfident();
    }
}
