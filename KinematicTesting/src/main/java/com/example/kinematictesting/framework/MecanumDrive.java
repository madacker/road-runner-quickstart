package com.example.kinematictesting.framework;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TurnConstraints;
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

public final class MecanumDrive {

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

    public MecanumDrive(Simulation simulation) {
        simulation.setKinematics(PARAMS);

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

    public void setDrivePowers(PoseVelocity2d powers) {
        setDrivePowers(powers, null);
    }

    /**
     * Power the motors according to the specified velocities. 'stickVelocity' is for controller
     * input and 'assistVelocity' is for computed driver assistance. The former is specified in
     * voltage values normalized from -1 to 1 (just like the regular DcMotor::SetPower() API)
     * whereas the latter is in inches/s or radians/s. Both types of velocities can be specified
     * at the same time in which case the velocities are added together (to allow assist and stick
     * control to blend together, for example).
     *
     * It's also possible to map the controller input to inches/s and radians/s instead of the
     * normalized -1 to 1 voltage range. You can reference MecanumDrive.PARAMS.maxWheelVel and
     * .maxAngVel to determine the range to specify. Note however that the robot can actually
     * go faster than Road Runner's PARAMS values so you would be unnecessarily slowing your
     * robot down.
     */
    public void setDrivePowers(
            // Manual power, normalized voltage from -1 to 1, robot-relative coordinates, can be null:
            PoseVelocity2d stickVelocity,
            // Computed power, inches/s and radians/s, field-relative coordinates, can be null:
            PoseVelocity2d assistVelocity)
    {
        PoseVelocity2d fieldVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (stickVelocity != null) {
            fieldVelocity = new PoseVelocity2d(new Vector2d(
                    stickVelocity.linearVel.x * MecanumDrive.PARAMS.maxWheelVel,
                    stickVelocity.linearVel.y * MecanumDrive.PARAMS.maxWheelVel),
                    stickVelocity.angVel * MecanumDrive.PARAMS.maxAngVel);
            fieldVelocity = pose.times(fieldVelocity); // Make it field-relative
        }
        if (assistVelocity != null) {
            fieldVelocity = new PoseVelocity2d(new Vector2d(
                    fieldVelocity.linearVel.x + assistVelocity.linearVel.x,
                    fieldVelocity.linearVel.y + assistVelocity.linearVel.y),
                    fieldVelocity.angVel + assistVelocity.angVel);
        }
        simulation.requestVelocity(fieldVelocity);
    }
}
