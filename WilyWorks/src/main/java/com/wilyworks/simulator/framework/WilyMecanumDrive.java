package com.wilyworks.simulator.framework;

import com.acmerobotics.dashboard.canvas.Canvas;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;

import java.util.LinkedList;

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
    Twist2dDual<Time> update() {
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

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] { deltaLinear.x, deltaLinearVel.x }),
                        new DualNum<>(new double[] { deltaLinear.y, deltaLinearVel.y })
                ),
                new DualNum<>(new double[] { deltaAng, deltaAngVel })
        );
    }
}

public final class WilyMecanumDrive {

    public static class Params {
        // path profile parameters (in inches)
        public double maxWheelVel = 60;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;
    }

    public static Params PARAMS = new Params();
    public final Localizer localizer;
    public Pose2d pose;
    public PoseVelocity2d poseVelocity; // Robot-relative, not field-relative
    Simulation simulation;

    public WilyMecanumDrive(Simulation simulation) {
        this.pose = new Pose2d(
                simulation.pose.position.x,
                simulation.pose.position.y,
                simulation.pose.heading.log());
        this.simulation = simulation;
        this.localizer = new Localizer(simulation);
    }

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();

        final boolean USE_POSE_EXPONTENTIAL = false;
        if (USE_POSE_EXPONTENTIAL) {
            // Use pose exponential refinement and conversion to field-relative:
            pose = pose.plus(twist.value());
        } else {
            // Use forward Euler integration for refinement and conversion:
            Twist2d robotTwist = twist.value();
            Vector2d fieldDelta = Localizer.transform(
                    robotTwist.line.x,
                    robotTwist.line.y,
                    pose.heading.log() + robotTwist.angle);
            pose = new Pose2d(
                    pose.position.x + fieldDelta.x,
                    pose.position.y + fieldDelta.y,
                    pose.heading.log() + robotTwist.angle);
        }

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        poseVelocity = twist.velocity().value();
        return poseVelocity;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas c, Pose2d t, double robotRadius) {
        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, robotRadius);

        Vector2d halfv = t.heading.vec().times(0.5 * robotRadius);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;
        drawRobot(c, t, ROBOT_RADIUS);
    }
}
