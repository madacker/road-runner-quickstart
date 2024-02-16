/**
 * This module is responsible for refinements to the odometry-based pose estimation.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.System.nanoTime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.annotation.SuppressLint;
import android.graphics.PointF;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

class DistanceResult {
    double measurement; // Measured distance
    int sensorIndex; // SENSOR_DESCRIPTORS index
    int segmentIndex; // Target WALL_SEGMENTS index
    double latency; // Latency, in seconds

    // These are computed:
    Point point; // Point computed from the measurement and most recent pose information
    boolean valid; // True if the sensor result was valid and used

    public DistanceResult(int sensorIndex, int segmentIndex, double latency) {
        this.sensorIndex = sensorIndex;
        this.segmentIndex = segmentIndex;
        this.latency = latency;
    }
}

class AprilTagResult {
    Pose2d pose; // Measured result
    int cameraIndex; // CAMERA_DESCRIPTORS index
    double latency; // Latency, in seconds
}

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

class Segment {
    public Point p1, p2;
    public Segment(double x1, double y1, double x2, double y2) {
        p1 = new Point(x1, y1);
        p2 = new Point(x2, y2);
    }
}

class Ray {
    public Point origin, direction;
    public Ray(Point origin, Point direction) {
        this.origin = origin;
        this.direction = direction;
    }
}

/**
 * Class for localizing distance sensor.
 */
class DistanceLocalizer {

    // Description of the distance sensor on the robot:
    static final SensorDescriptor[] SENSOR_DESCRIPTORS = {
        new SensorDescriptor("distance2", new Point(-3, -1), Math.PI)
    };

    // This is list is maintained one-to-one with SENSOR_DESCRIPTORS:
    ArrayList<DistanceSensor> sensorHardware = new ArrayList<DistanceSensor>(); // @@@ Initialize

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

    // Structure for tracking sensor placements on the robot:
    static class SensorDescriptor {
        // Name of the sensor in the robot configuration:
        String name;

        // The position is in inches relative to the center of rotation, with the robot facing
        // forward.
        Point offset;

        // The orientation of the sensor at the specified position, in radians. 0 points
        // straight ahead:
        public double theta;

        public SensorDescriptor(String name, Point offset, double theta) {
            this.name = name;
            this.offset = offset;
            this.theta = theta;
        }
    }

    int currentSensorIndex; // Index into sensorPlacements of the current sensor
    double lastReadTime; // Time of the last read of a sensor

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
    DistanceResult update(Pose2d pose) {
        double time = Globals.time();
        if (time - lastReadTime < READ_INTERVAL)
            return null;

        // Create an eligibility list of all sensors that point to a wall segment. We'll choose
        // the best of it next.
        ArrayList<DistanceResult> eligibleSensors = new ArrayList<>();
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
                        eligibleSensors.add(new DistanceResult(sensorIndex, segmentIndex, LATENCY));
                    }
                }
            }
        }

        // Choose what sensor to use:
        DistanceResult result;
        int eligibleCount = eligibleSensors.size();
        if (eligibleCount == 0) {
            // There are no good candidates so simply do a different sensor than last time:
            currentSensorIndex++;
            if (currentSensorIndex >= SENSOR_DESCRIPTORS.length)
                currentSensorIndex = 0;
            result = new DistanceResult(currentSensorIndex, 0, LATENCY);
        } else {
            // If more than one eligible sensor, choose one that's different from the one
            // we used last time:
            result = eligibleSensors.get(0);
            if ((result.sensorIndex == currentSensorIndex) && (eligibleCount > 1))
                result = eligibleSensors.get(1);
        }
        currentSensorIndex = result.sensorIndex;

        // Query the hardware:
        result.measurement = sensorHardware.get(result.sensorIndex).getDistance(DistanceUnit.INCH);
        return result;
    }

    /**
     * Given a starting pose and a measured distance, calculate a corrected pose.
     */
    static public Pose2d localize(Pose2d pose, DistanceResult distance) {
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

                pose = new Pose2d(new Vector2d(newPosition.x, newPosition.y), pose.heading);
                valid = true;
            }
        }

        // Update telemetry:
        distance.valid = valid;
        distance.point = new Point(
                pose.position.x + distance.measurement * Math.cos(pose.heading.log()),
                pose.position.y + distance.measurement * Math.sin(pose.heading.log()));

        return pose;
    }
}

class AprilTagLocalizer {
    Pose2d pose;
    double latency; // @@@ Seconds for the camera currently being used?

    // @@@ Update pose and latency:
    boolean update() { return false; }
}

class Poser {
    // Keep the history around for this many seconds:
    private final static double HISTORY_DURATION = 2.0;

    private Pose2d pose;
    private PoseVelocity2d poseVelocity;
    private Vector2d residual;

    private LinkedList<HistoricalRecord> history = new LinkedList<>(); // Newest first

    private Localizer odometryLocalizer;
    private DistanceLocalizer distanceLocalizer;
    private AprilTagLocalizer aprilTagLocalizer;

    class HistoricalRecord {
        double time; // Time, in seconds
        Twist2d twist; // Odometry twist
        Pose2d postOdometryPose; // Pose after having applied the twist but before distances
        ArrayList<DistanceResult> distances; // Distance sensor distances for this time interval, if any
    }

    static Pose2d copyPose(Pose2d pose) {
        return new Pose2d(
                new Vector2d(pose.position.x, pose.position.y),
                new Rotation2d(pose.heading.real, pose.heading.imag));
    }

    // Update an April-tag or a distance record for a particular time in the past and then
    // recompute all the history back to the present:
    private void changeHistory(double time, Pose2d newAprilTagPose, DistanceResult newDistance) {
        if (history.size() == 0)
            return; // ===>

        // Find the first record older than the target:
        ListIterator<HistoricalRecord> iterator = history.listIterator();
        HistoricalRecord iteratorRecord;
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
        pose = (newAprilTagPose != null) ? newAprilTagPose : iteratorRecord.postOdometryPose;
        do {
            // Apply the odometry twist:
            pose = pose.plus(iteratorRecord.twist);

            // Update the post-odometry pose:
            iteratorRecord.postOdometryPose = pose;

            // Apply the distances:
            for (DistanceResult distance: iteratorRecord.distances) {
                pose = DistanceLocalizer.localize(pose, distance);
            }

            // Go to the next newer record:
            iteratorRecord = iterator.previous();
        } while (iteratorRecord != null);
    }

    // Update the pose estimate:
    public void update() {
        double time = Globals.time();
        Twist2dDual<Time> dualTwist = odometryLocalizer.update();
        poseVelocity = dualTwist.velocity().value();

        // Update the current pose from odometry and create a tracking record:
        HistoricalRecord record = new HistoricalRecord();
        record.time = time;
        record.twist = dualTwist.value();
        pose = pose.plus(record.twist);
        record.postOdometryPose = copyPose(pose);
        history.addFirst(record);

        // Process April Tags and distance sensor data:
        if (aprilTagLocalizer.update()) {
            changeHistory(time - aprilTagLocalizer.latency, aprilTagLocalizer.pose, null);
        }
        DistanceResult distance = distanceLocalizer.update(pose);
        if (distance != null) {
            changeHistory(time - distance.latency, null, distance);
        }

        // Delete old history records:
        while (time - history.getLast().time > HISTORY_DURATION) {
            history.removeLast();
        }
    }

    // Return the current pose:
    public Pose2d getPose() {
        return copyPose(pose);
    }

    // Return the current pose velocity:
    public PoseVelocity2d getPoseVelocity() {
        return new PoseVelocity2d(new Vector2d(poseVelocity.linearVel.x, poseVelocity.linearVel.y),
                poseVelocity.angVel);
    }
}

/**
 * Maintain a history of residuals and apply a sliding window averaging filter.
 */
class ResidualFilter {
    static final double POSITION_WINDOW_SIZE = 5; // Count of points
    static final double POSITION_WATCHDOG_DURATION = 0.2; // Seconds
    static final double HEADING_WINDOW_DURATION = 20.0; // Seconds
    static final int LOCKED_IN_HEADING_COUNT = 10; // Need 10 trusted heading reading before locking in

    static final double MAX_POSITION_CHANGE_RATE = 18; // Inches per second
    static final double MAX_HEADING_CHANGE_RATE = Math.toRadians(1); // Degrees per second

    private double previousTime; // Time of the most recent filter() call
    private double trustedHeadingCount = 0; // Active count of trusted headings so far

    public boolean lockedIn; // True if we highly trust the current headings

    // Time-stamped storage structure for positions and headings:
    static class Storage<T> {
        T datum;
        double time;
        Storage(T datum) {
            this.datum = datum;
            this.time = time();
        }
    }

    private LinkedList<Storage<Vector2d>> positionResiduals = new LinkedList<>();
    private LinkedList<Storage<Double>> headingResiduals = new LinkedList<>();

    static double time() { return nanoTime() * 1e-9; }

    // If lockedHeading is true then the rate of change of the heading will be tightly
    // clamped from the very beginning:
    ResidualFilter(boolean lockedIn) {
        this.lockedIn = lockedIn;
        previousTime = time();
    }

    double normalizeAngle(double angle) {
        while (angle >= Math.PI)
            angle -= 2 * Math.PI;
        while (angle < -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    void recordSample(Pose2d currentPose, Pose2d sampledPose, boolean trustedHeading) {
        // Compute the residuals for the new pose and add them to the histories:
        Vector2d positionResidual = sampledPose.position.minus(currentPose.position);
        positionResiduals.add(new Storage(positionResidual));

        // Even an untrusted heading is better than no heading at all:
        if ((trustedHeading) || (headingResiduals.size() == 0)) {
            double headingResidual = normalizeAngle(sampledPose.heading.log() - currentPose.heading.log());
            headingResiduals.add(new Storage(headingResidual));

            trustedHeadingCount++;
            if (trustedHeadingCount > LOCKED_IN_HEADING_COUNT)
                lockedIn = true;
        }
    }

    Pose2d filter(Pose2d currentPose) {
        // Track the current time and the delta-t from the previous filter operation:
        double time = time();
        double dt = time - previousTime;
        previousTime = time;

        // If it's been some time since we got a new position residual, toss them all. We don't
        // want to age them out one at a time because that simply biases everything to the
        // most recent one:
        if ((positionResiduals.size() > 0) && (time - positionResiduals.getLast().time > POSITION_WATCHDOG_DURATION))
            positionResiduals.clear();

        // Remove all aged-out residuals from the histories:
        while ((positionResiduals.size() > 0) && (positionResiduals.size() > POSITION_WINDOW_SIZE))
            positionResiduals.removeFirst();
        while ((headingResiduals.size() > 0) && (time - headingResiduals.get(0).time > HEADING_WINDOW_DURATION))
            headingResiduals.removeFirst();

        // Compute the averages of all the residuals:
        Vector2d aggregatePositionResidual = new Vector2d(0, 0);
        double aggregateHeaderResidual = 0;

        for (Storage position: positionResiduals) {
            aggregatePositionResidual = aggregatePositionResidual.plus((Vector2d) position.datum);
        }
        for (Storage heading: headingResiduals) {
            aggregateHeaderResidual = aggregateHeaderResidual + (Double) heading.datum;
        }

        if (positionResiduals.size() != 0)
            aggregatePositionResidual = aggregatePositionResidual.div(positionResiduals.size());
        if (headingResiduals.size() != 0)
            aggregateHeaderResidual = aggregateHeaderResidual / headingResiduals.size();

        // Compute how much to correct the current position and heading:
        Vector2d positionCorrection = new Vector2d(aggregatePositionResidual.x, aggregatePositionResidual.y);
        double headingCorrection = aggregateHeaderResidual;

        // Once we're locked in, clamp the rate of any changes:
        if (lockedIn) {
            double distance = Math.hypot(aggregatePositionResidual.x, aggregatePositionResidual.y);
            if (distance > dt * MAX_POSITION_CHANGE_RATE) {
                double clampFactor = dt * MAX_POSITION_CHANGE_RATE / distance;
                positionCorrection = positionCorrection.times(clampFactor);
            }
            if (Math.abs(headingCorrection) > dt * MAX_HEADING_CHANGE_RATE) {
                headingCorrection = dt * Math.signum(headingCorrection) * MAX_HEADING_CHANGE_RATE;
            }
        }

        // Negatively offset everything in the history to account for the offset we're adding:
        for (Storage position: positionResiduals) {
            position.datum = positionCorrection.minus((Vector2d) position.datum);
        }
        for (Storage heading: headingResiduals) {
            heading.datum = headingCorrection - (double) heading.datum;
        }

        // Correct the current pose to create the new pose:
        return new Pose2d(
                currentPose.position.plus(positionCorrection),
                currentPose.heading.log() + headingCorrection);
    }
}

/**
 * Pose refiner. Leverages AprilTags and the distance sensor to improve the accuracy of the
 * estimated pose.
 */
public class PoseEstimator {
    // True if doing April Tags latency calibration. Should be false normally:
    private static final boolean LATENCY_CALIBRATION = false;

    // Camera characteristics:
    private static final String CAMERA_NAME = "webcam2";
    private static final double CAMERA_OFFSET_Y = 8.0; // Camera location on the robot (assuming it's facing forward)
    private static final double CAMERA_OFFSET_X = -5.75;
    private static final double CAMERA_ORIENTATION = Math.PI;
    private static final int CAMERA_PIXEL_WIDTH = 1280;
    private static final int CAMERA_PIXEL_HEIGHT = 720;
    private static final double CAMERA_FX = 906.940247073; // Set to -1.0 to use FTC defaults
    private static final double CAMERA_FY = 906.940247073;
    private static final double CAMERA_CX = 670.833056673;
    private static final double CAMERA_CY = 355.34234068;
    private static final double CAMERA_LATENCY = 0.19; // Seconds

    // Offset distance from sensor to center of robot:
    private static final double DISTANCE_SENSOR_OFFSET = 1;

    // Run-time state:
    private ResidualFilter residualFilter;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DistanceSensor distanceSensor;

    // Informational state tracking:
    private double windowStart; // In seconds
    private int windowCount;
    private double fps;
    private String poseStatus = "";
    private double pipelineLatency = 0; // Milliseconds
    private TimeSplitter distanceTimer = TimeSplitter.create("getDistance");

    // Structure defining the location of April Tags:
    static class AprilTagLocation {
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

    public PoseEstimator(HardwareMap hardwareMap, boolean lockedInPose) {
        residualFilter = new ResidualFilter(lockedInPose);
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
        aprilTag.setDecimation(3);

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

    private final LinkedList<PointF> distanceHistory = new LinkedList<>();

    private void visualizeDistance(Pose2d odometryPose) {
        distanceTimer.startSplit();
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        distanceTimer.endSplit();

        if (distance >= 0) {
            distance += DISTANCE_SENSOR_OFFSET;

            double theta = odometryPose.heading.log() + Math.PI; // +180 degrees because facing backwards
            double x = odometryPose.position.x + distance * Math.cos(theta);
            double y = odometryPose.position.y + distance * Math.sin(theta);

            distanceHistory.add(new PointF((float) x, (float) y));
            if (distanceHistory.size() > 100) {
                distanceHistory.removeFirst();
            }
        }

        Globals.canvas.setStrokeWidth(1);
        Globals.canvas.setFill("#808080");
        for (PointF point: distanceHistory) {
            Globals.canvas.fillRect(point.x, point.y, 1, 1);
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
        return Math.hypot(a.position.x - b.position.x, a.position.y - b.position.y);
    }

    /**
     *  Return a refined pose based on April Tags and the distance sensor.
     */
    @SuppressLint("DefaultLocale")
    public Pose2d refinePose(Pose2d currentPose, MecanumDrive drive) {

        // We arbitrarily decide that a pose has to be within this many inches to be reliable:
        final double FINE_EPSILON = 5.0;
        final double COARSE_EPSILON = 10.0;

        visualizeDistance(currentPose);

        ArrayList<VisionPose> visionPoses = new ArrayList<>();
        double minDistance = Float.MAX_VALUE;

        List<AprilTagDetection> tagDetections = aprilTag.getFreshDetections();
        if (tagDetections != null) {
            for (AprilTagDetection detection : tagDetections) {
                pipelineLatency = (nanoTime() - tagDetections.get(0).frameAcquisitionNanoTime) * 10e-6;
                if (detection.metadata != null) {
                    AprilTagLocation tag = getTag(detection);
                    if (tag != null) {
                        Pose2d datedVisionPose = computeRobotPose(detection, tag);
                        Pose2d visionPose = drive.applyTwistHistory(datedVisionPose, CAMERA_LATENCY);

                        Globals.canvas.setStroke(tag.large ? "#00ff00" : "#c0c000");
                        MecanumDrive.drawRobot(Globals.canvas, visionPose, 7);

                        double distance = Math.hypot(
                                currentPose.position.x - visionPose.position.x,
                                currentPose.position.y - visionPose.position.y);

                        minDistance = Math.min(minDistance, distance);
                        visionPoses.add(new VisionPose(visionPose, distance, tag));
                    }
                }
            }
        }

        // Track the April Tag FPS:
        double intervalDuration = Globals.time() - windowStart;
        if (intervalDuration > 1.0) {
            fps = windowCount / intervalDuration;
            windowStart = Globals.time();
            windowCount = 0;
        }
        if (tagDetections != null) {
            windowCount++;
        }

        // This will be the vision pose that we recommend to update the current pose:
        VisionPose visionPose = null;
        int poseCount = visionPoses.size();
        boolean excellentPose = false;

        if (LATENCY_CALIBRATION) {
            // When doing April Tags latency calibration, return success as soon as we get
            // a single pose, no matter how bad:
            visionPose = (visionPoses.size() != 0) ? visionPoses.get(0) : null;
        } else {
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
                            visionPose = visionPoses.get(i);
                            poseStatus = String.format("Excellent vision 3-pose, min %.2f, max %.2f", min, maxNeighborDistance);
                            excellentPose = true;
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
        }

        // Update some status:
        Globals.telemetry.addLine(String.format("Vision FPS: %.2f (ours), %.2f (theirs)", fps, visionPortal.getFps()));
        Globals.telemetry.addLine(String.format("Vision pose count: %d, ms: %.2f", visionPoses.size(), pipelineLatency));
        Globals.telemetry.addLine(poseStatus);

        if (visionPose != null) {
            // Add the new vision pose to our filter history:
            residualFilter.recordSample(currentPose, visionPose.pose, excellentPose);
        }

        return residualFilter.filter(currentPose);
    }
}
