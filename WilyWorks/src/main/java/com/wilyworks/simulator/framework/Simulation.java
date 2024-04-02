package com.wilyworks.simulator.framework;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.example.kinematictesting.framework.DashboardCanvas;
import com.example.kinematictesting.framework.DashboardWindow;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.wilyworks.simulator.framework.WilyTelemetry;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsEnvironment;
import java.awt.Image;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Transparency;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

class Localizer {
    Simulation simulation;
    Pose2d previousPose;
    PoseVelocity2d previousVelocity;

    Localizer(Simulation simulation) {
        this.simulation = simulation;
        this.previousPose = simulation.pose;
        this.previousVelocity = simulation.poseVelocity;
    }
    static Vector2d transform(double x, double y, double theta) {
        return new Vector2d(x * Math.cos(theta) - y * Math.sin(theta),
                x * Math.sin(theta) + y * Math.cos(theta));
    }
    double[] update() {
        double deltaAng = simulation.pose.heading.log() - previousPose.heading.log();
        double deltaAngVel = simulation.poseVelocity.angVel - previousVelocity.angVel;

        // Transform from field-absolute position to robot-relative position:
        double robotAngle = simulation.pose.heading.log();
        Vector2d deltaLinear = transform(
                simulation.pose.position.x - previousPose.position.x,
                simulation.pose.position.y - previousPose.position.y,
                -robotAngle);
        Vector2d deltaLinearVel = transform(
                simulation.poseVelocity.linearVel.x - previousVelocity.linearVel.x,
                simulation.poseVelocity.linearVel.y - previousVelocity.linearVel.y,
                -robotAngle);

        previousPose = simulation.pose;
        previousVelocity = simulation.poseVelocity;

        return new double[]{deltaLinear.x, deltaLinear.y, deltaAng,
            deltaLinearVel.x, deltaLinearVel.y, deltaAngVel};
    }
}

/**
 * Kinematic simulation for the robot's movement.
 */
public class Simulation {
    public static class Kinematics {
        // path profile parameters (in inches)
        public double maxWheelVel = 60;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;
    }

    public Pose2d pose = new Pose2d(-48, 0, Math.toRadians(90)); // Robot's true pose
    public PoseVelocity2d poseVelocity = new PoseVelocity2d(new Vector2d(0, 60), Math.toRadians(0)); // Robot's true pose velocity
    public Dimension robotSize = new Dimension(16, 18); // Size in inches of user's robot
    public DashboardCanvas canvas; // Canvas for the entire window frame

    private Localizer localizer;
    private Kinematics kinematics = new Kinematics(); // Kinematic parameters for the simulation
    private PoseVelocity2d requestedVelocity; // Velocity requested by MecanumDrive

    public Simulation() {
        localizer = new Localizer(this);
    }

    // Get the current time, in seconds:
    static double time() {
        return nanoTime() * 1e-9;
    }

    // Move the robot in the requested direction via kinematics:
    public void advance(double dt) {
        // Request a stop if no new velocity has been requested:
        if (requestedVelocity == null)
            requestedVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

        // Handle the rotational velocity:
        double currentAngular = poseVelocity.angVel;
        double requestedAngular = requestedVelocity.angVel;
        double deltaAngular = requestedAngular - currentAngular;
        if (deltaAngular >= 0) {
            // Increase the angular velocity:
            currentAngular += kinematics.maxAngAccel * dt;
            currentAngular = Math.min(currentAngular, kinematics.maxAngVel);
            currentAngular = Math.min(currentAngular, requestedAngular);
        } else {
            // Decrease the angular velocity:
            currentAngular -= kinematics.maxAngAccel * dt; // maxAngAccel is positive
            currentAngular = Math.max(currentAngular, -kinematics.maxAngVel);
            currentAngular = Math.max(currentAngular, requestedAngular);
        }

        // Handle the linear velocity:
        double currentLinearX = poseVelocity.linearVel.x;
        double currentLinearY = poseVelocity.linearVel.y;
        double requestedLinearX = requestedVelocity.linearVel.x;
        double requestedLinearY = requestedVelocity.linearVel.y;

        double currentVelocity = Math.hypot(currentLinearX, currentLinearY);
        double requestedVelocity = Math.hypot(requestedLinearX, requestedLinearY);
        double currentAngle = Math.atan2(currentLinearY, currentLinearX); // Rise over run
        double requestedAngle = Math.atan2(requestedLinearY, requestedLinearX);

        WilyTelemetry.instance.addData("requestedVelocity", requestedVelocity); // @@@

        // If the requested velocity is close to zero then its angle is rather undetermined.
        // Use the current angle in that case:
        if (Math.abs(requestedVelocity) == 0)
            requestedAngle = currentAngle;
        double theta = requestedAngle - currentAngle; // Angle from current to requested

        // Clamp to the maximum allowable velocities:
        currentVelocity = Math.min(currentVelocity, kinematics.maxWheelVel);
        requestedVelocity = Math.min(requestedVelocity, kinematics.maxWheelVel);

        // Perpendicular velocity is the current velocity component away from
        // the requested velocity. We reduce this by the deceleration:
        double perpVelocity = Math.sin(theta) * currentVelocity;
        if (perpVelocity >= 0) {
            perpVelocity += kinematics.minProfileAccel * dt; // minProfileAccel is negative
            perpVelocity = Math.max(perpVelocity, 0);
        } else {
            perpVelocity -= kinematics.minProfileAccel * dt;
            perpVelocity = Math.min(perpVelocity, 0);
        }

        // Parallel velocity is the current velocity component in the same direction
        // as the requested velocity. Accelerate or decelerate to match our parallel
        // velocity with the request:
        double parallelVelocity = Math.cos(theta) * currentVelocity;
        double parallelDelta = requestedVelocity - parallelVelocity;

        // We now know our perpendicular velocity and we know the maximum allowable
        // velocity so our maximum parallel velocity is remainder. Note that we're
        // guaranteed that won't try to do the square root of a negative:
        double maxParallelVelocity =
                Math.sqrt(Math.pow(kinematics.maxWheelVel, 2) - Math.pow(perpVelocity, 2));

        if (parallelDelta >= 0) { // Increase the parallel velocity
            parallelVelocity += kinematics.maxProfileAccel * dt;
            parallelVelocity = Math.min(parallelVelocity, maxParallelVelocity);
            parallelVelocity = Math.min(parallelVelocity, requestedVelocity);
        } else { // Decrease the parallel velocity:
            parallelVelocity -= kinematics.maxProfileAccel * dt; // maxProfileAccel is positive
            parallelVelocity = Math.max(parallelVelocity, -maxParallelVelocity);
            parallelVelocity = Math.max(parallelVelocity, requestedVelocity);
        }
        currentLinearX = Math.cos(requestedAngle) * parallelVelocity
                       + Math.cos(requestedAngle - Math.PI / 2) * perpVelocity;
        currentLinearY = Math.sin(requestedAngle) * parallelVelocity
                       + Math.sin(requestedAngle - Math.PI / 2) * perpVelocity;

        double x = pose.position.x + dt * currentLinearX;
        double y = pose.position.y + dt * currentLinearY;

        // Wrap the coordinates for now:
        if (x > 72.0)
            x -= 144.0;
        if (x <= -72.0)
            x += 144.0;
        if (y > 72.0)
            y -= 144.0;
        if (y <= -72.0)
            y += 144.0;

        // Update our official pose and velocity:
        pose = new Pose2d(x, y, pose.heading.log() + dt * currentAngular);
        poseVelocity = new PoseVelocity2d(new Vector2d(currentLinearX, currentLinearY), currentAngular);
    }

    // Power the motors according to the specified velocities. 'stickVelocity' is for controller
    // input and 'assistVelocity' is for computed driver assistance. The former is specified in
    // voltage values normalized from -1 to 1 (just like the regular DcMotor::SetPower() API)
    // whereas the latter is in inches/s or radians/s. Both types of velocities can be specified
    // at the same time in which case the velocities are added together (to allow assist and stick
    // control to blend together, for example).
    //
    // It's also possible to map the controller input to inches/s and radians/s instead of the
    // normalized -1 to 1 voltage range. You can reference MecanumDrive.PARAMS.maxWheelVel and
    // .maxAngVel to determine the range to specify. Note however that the robot can actually
    // go faster than Road Runner's PARAMS values so you would be unnecessarily slowing your
    // robot down.
    public void setDrivePowers(
            // Manual power, normalized voltage from -1 to 1, robot-relative coordinates, can be null:
            PoseVelocity2d stickVelocity,
            // Computed power, inches/s and radians/s, field-relative coordinates, can be null:
            PoseVelocity2d assistVelocity)
    {
        PoseVelocity2d fieldVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (stickVelocity != null) {
            fieldVelocity = new PoseVelocity2d(new Vector2d(
                    stickVelocity.linearVel.x * kinematics.maxWheelVel,
                    stickVelocity.linearVel.y * kinematics.maxWheelVel),
                    stickVelocity.angVel * kinematics.maxAngVel);
            fieldVelocity = pose.times(fieldVelocity); // Make it field-relative
        }
        if (assistVelocity != null) {
            fieldVelocity = new PoseVelocity2d(new Vector2d(
                    fieldVelocity.linearVel.x + assistVelocity.linearVel.x,
                    fieldVelocity.linearVel.y + assistVelocity.linearVel.y),
                    fieldVelocity.angVel + assistVelocity.angVel);
        }
        this.requestedVelocity = fieldVelocity;
    }

    // Entry point to get the current localizer position:
    public double[] localizerUpdate() { return localizer.update(); }

    // Entry point to set the pose and velocity, both in field coordinates:
    public void setPose(Pose2d pose, PoseVelocity2d poseVelocity) {
        this.pose = pose;
        this.poseVelocity = poseVelocity;
        this.localizer = new Localizer(this);
    }
}
