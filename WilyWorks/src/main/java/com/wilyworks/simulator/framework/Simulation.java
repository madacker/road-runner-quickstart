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

class Field {
    // Make the field view 720x720 pixels but inset the field surface so that there's padding
    // all around it:
    final int FIELD_VIEW_DIMENSION = 720;
    final int FIELD_SURFACE_DIMENSION = 480;

    // These are derived from the above to describe the field rendering:
    final int FIELD_INSET = (FIELD_VIEW_DIMENSION - FIELD_SURFACE_DIMENSION) / 2;
    final Rectangle FIELD_VIEW = new Rectangle(0, 0, FIELD_VIEW_DIMENSION, FIELD_VIEW_DIMENSION);

    // Robot dimensions:
    final int ROBOT_IMAGE_WIDTH = 128;
    final int ROBOT_IMAGE_HEIGHT = 128;

    Simulation simulation;
    Image backgroundImage;
    BufferedImage robotImage;

    Field(Simulation simulation) {
        this.simulation = simulation;
        ClassLoader classLoader = Thread.currentThread().getContextClassLoader();

        InputStream stream = classLoader.getResourceAsStream("background/season-2023-centerstage/field-2023-juice-dark.png");
        if (stream != null) {
            try {
                backgroundImage = ImageIO
                        .read(stream)
                        .getScaledInstance(FIELD_SURFACE_DIMENSION, FIELD_SURFACE_DIMENSION, Image.SCALE_SMOOTH);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
        initializeRobotImage();
    }

    // Round to an integer:
    static int round(double value) {
        return (int) Math.round(value);
    }

    // Initialize the robot image bitmap:
    private void initializeRobotImage() {
        final int OPACITY = round(255 * 0.8);
        final double WHEEL_PADDING_X = 0.05;
        final double WHEEL_PADDING_Y = 0.05;
        final double WHEEL_WIDTH = 0.2;
        final double WHEEL_HEIGHT = 0.3;
        final double DIRECTION_LINE_WIDTH = 0.05;
        final double DIRECTION_LINE_HEIGHT = 0.4;

        GraphicsConfiguration config =
                GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().getDefaultConfiguration();
        robotImage = config.createCompatibleImage(ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT, Transparency.TRANSLUCENT);

        Graphics2D g = robotImage.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        // Draw the body:
        g.setColor(new Color(0xe5, 0x3e, 0x3d, OPACITY));
        g.fillRect(0, 0, ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT);

        // Draw the wheels:
        g.setColor(new Color(0x74, 0x2a, 0x2a, OPACITY));
        g.fillRect(
                round(WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH), round(WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(ROBOT_IMAGE_WIDTH - WHEEL_WIDTH * ROBOT_IMAGE_WIDTH - WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT), round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH),
                round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(ROBOT_IMAGE_WIDTH - WHEEL_WIDTH * ROBOT_IMAGE_WIDTH - WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT - WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT - WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));
        g.fillRect(
                round(WHEEL_PADDING_X * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT - WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT - WHEEL_PADDING_Y * ROBOT_IMAGE_HEIGHT),
                round(WHEEL_WIDTH * ROBOT_IMAGE_WIDTH), round(WHEEL_HEIGHT * ROBOT_IMAGE_HEIGHT));

        // Draw the direction indicator:
        g.setColor(new Color(0x74, 0x2a, 0x2a));
        g.fillRect(round(ROBOT_IMAGE_WIDTH / 2.0 - DIRECTION_LINE_WIDTH * ROBOT_IMAGE_WIDTH / 2.0), 0,
                round(DIRECTION_LINE_WIDTH * ROBOT_IMAGE_WIDTH),
                round(ROBOT_IMAGE_HEIGHT * DIRECTION_LINE_HEIGHT));
    }

    // Render just the robot:
    void renderRobot(Graphics2D g) {
        AffineTransform imageTransform = new AffineTransform();
        imageTransform.translate(simulation.pose.position.x, simulation.pose.position.y);
        imageTransform.scale(1.0 / ROBOT_IMAGE_WIDTH,1.0 / ROBOT_IMAGE_HEIGHT);
        imageTransform.rotate(simulation.pose.heading.log() + Math.toRadians(90));
        imageTransform.scale(simulation.robotSize.width, simulation.robotSize.height);
        imageTransform.translate(-ROBOT_IMAGE_HEIGHT / 2.0, -ROBOT_IMAGE_HEIGHT / 2.0);
        g.drawImage(robotImage, imageTransform, null);
    }

    // Render the field, the robot, and the field overlay:
    void render(Graphics2D g) {
        // Lay down the background image without needing a transform:
        g.drawImage(backgroundImage, FIELD_VIEW.x + FIELD_INSET, FIELD_VIEW.y + FIELD_INSET, null);

        // Prime the viewport/transform and the clipping for field and overlay rendering:
        AffineTransform oldTransform = g.getTransform();
        g.setClip(FIELD_VIEW.x, FIELD_VIEW.y, FIELD_VIEW.width, FIELD_VIEW.height);
        g.transform(new AffineTransform(
                FIELD_SURFACE_DIMENSION / 144.0, 0,
                0, -FIELD_SURFACE_DIMENSION / 144.0,
                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET,
                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET));

        renderRobot(g);
        if (FtcDashboard.fieldOverlay != null)
            FtcDashboard.fieldOverlay.render(g);

        // Restore:
        g.setTransform(oldTransform);

        // Draw some debug text:
        g.setColor(new Color(0xffffff));
        g.drawString("Hello kinematic world!", 20, 20);
    }
}

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

    private Field field;
    private Localizer localizer;
    private Kinematics kinematics = new Kinematics(); // Kinematic parameters for the simulation
    private PoseVelocity2d requestedVelocity; // Velocity requested by MecanumDrive
    private double lastUpdateTime = time(); // Time of last update() call, in seconds

    public Simulation() {
        DashboardWindow dashboardWindow = new DashboardWindow("FTC Dashboard", 1280, 720);
        dashboardWindow.setVisible(true);
        canvas = dashboardWindow.getCanvas();
        field = new Field(this);
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

    // Advance the simulation and update the field view. deltaT is specified in seconds.
    public void update(double deltaT) {
        // If delta-t is zero then use the real-time clock:
        double time = time();
        if (deltaT <= 0) {
            deltaT = time - lastUpdateTime;
        }
        lastUpdateTime = time;

        // Advance the simulation:
        advance(deltaT);

        // All Graphics objects can be cast to Graphics2D:
        Graphics2D g = (Graphics2D) canvas.getBufferStrategy().getDrawGraphics();

        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        field.render(g);

        g.dispose();
        canvas.getBufferStrategy().show();
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
}
