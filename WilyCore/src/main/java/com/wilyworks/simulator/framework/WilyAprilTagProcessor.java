package com.wilyworks.simulator.framework;

import com.wilyworks.common.WilyWorks;

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
    WilyWorks.Config.Camera wilyCamera;

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
        return null;
    }

    @Override
    public ArrayList<AprilTagDetection> getDetections() {
        return new ArrayList<>(); // ### Empty detection list
    }
}
