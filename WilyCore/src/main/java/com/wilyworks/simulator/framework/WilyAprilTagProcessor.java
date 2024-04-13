package com.wilyworks.simulator.framework;

import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * Wily Works implementation of the AprilTagProcessor interface.
 */
public class WilyAprilTagProcessor extends AprilTagProcessor {
    final double MAX_FPS = 20.0; // Maximum FPS

    WilyWorks.Config.Camera wilyCamera; // Describes the camera placement on the robot, if any
    double lastDetectionTime = WilyCore.time(); // Time of the last fresh detection
    ArrayList<AprilTagDetection> lastDetections = new ArrayList<>(); // A copy of the last detections provided

    public WilyAprilTagProcessor(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily, int threads) {
    }

    // WilyVisionPortal calls 'initialize' to associate the camera:
    public void initialize(WilyWorks.Config.Camera wilyCamera) {
        this.wilyCamera = wilyCamera;
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
        if (wilyCamera == null)
            return null;

        // Cap the fresh detections to the maximum frame-rate:
        if (WilyCore.time() - lastDetectionTime < 1 / MAX_FPS)
            return null;

        lastDetections = null;
        lastDetectionTime = WilyCore.time();
        return null;
    }

    @Override
    public ArrayList<AprilTagDetection> getDetections() {
         ArrayList<AprilTagDetection> detections = getFreshDetections();
        if (detections == null)
            detections = lastDetections;
         return detections;
    }
}
