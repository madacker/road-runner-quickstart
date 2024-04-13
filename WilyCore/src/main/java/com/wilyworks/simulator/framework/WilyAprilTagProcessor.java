package com.wilyworks.simulator.framework;

import com.acmerobotics.roadrunner.Pose2d;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;
import com.wilyworks.simulator.helpers.Globals;
import com.wilyworks.simulator.helpers.Point;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * Wily Works implementation of the AprilTagProcessor interface.
 */
public class WilyAprilTagProcessor extends AprilTagProcessor {
    final double MAX_FPS = 20.0; // Maximum FPS
    WilyWorks.Config.Camera cameraDescriptor; // Describes the camera placement on the robot, if any
    double lastDetectionTime = WilyCore.time(); // Time of the last fresh detection
    ArrayList<AprilTagDetection> lastDetections = new ArrayList<>(); // A copy of the last detections provided
    AprilTagMetadata[] tags = AprilTagGameDatabase.getCenterStageTagLibrary().getAllTags(); // Tag database

    public WilyAprilTagProcessor(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily, int threads) {
    }

    // WilyVisionPortal calls 'initialize' to associate the camera:
    public void initialize(WilyWorks.Config.Camera wilyCamera) {
        this.cameraDescriptor = wilyCamera;
    }




    @Override
    public void setDecimation(float decimation) {}

    @Override
    public void setPoseSolver(PoseSolver poseSolver) {}

    @Override
    public int getPerTagAvgPoseSolveTime() {
        return 0;
    }

    @Override
    public ArrayList<AprilTagDetection> getFreshDetections() {
        if (cameraDescriptor == null)
            return null;

        // Cap the fresh detections to the maximum frame-rate:
        if (WilyCore.time() - lastDetectionTime < 1 / MAX_FPS)
            return null;

        // @@@ Check if enabled

        ArrayList<AprilTagDetection> detections = new ArrayList<>();
        Pose2d pose = WilyCore.getPose(cameraDescriptor.latency);
        for (AprilTagMetadata tag: tags) {
            Point vectorToTag = new Point(tag.fieldPosition).subtract(new Point(pose.position));
            double tagAngle = vectorToTag.atan2();

            double cameraAngle = pose.heading.log() + cameraDescriptor.orientation;
            double halfFov = (cameraDescriptor.fieldOfView / 2) * 1.3;
            double rightAngle = cameraAngle - halfFov;
            double leftAngle = cameraAngle + halfFov;
            double deltaRight = Globals.normalizeAngle(tagAngle - rightAngle);
            double deltaLeft = Globals.normalizeAngle(leftAngle - tagAngle);
            if ((deltaRight > 0) && (deltaLeft > 0)) {
                // @@@ We got a hit!
            }
        }

        lastDetections = detections;
        lastDetectionTime = WilyCore.time();
        return detections;
    }

    @Override
    public ArrayList<AprilTagDetection> getDetections() {
         ArrayList<AprilTagDetection> detections = getFreshDetections();
        if (detections == null)
            detections = lastDetections;
         return detections;
    }
}
