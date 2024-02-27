package com.example.kinematictesting.framework;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

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

import javax.imageio.ImageIO;

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

        try {
            backgroundImage = ImageIO
                    .read(classLoader.getResourceAsStream("background/season-2023-centerstage/field-2023-juice-dark.png"))
                    .getScaledInstance(FIELD_SURFACE_DIMENSION, FIELD_SURFACE_DIMENSION, Image.SCALE_SMOOTH);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        initializeRobotImage();
    }

    static int round(double value) {
        return (int) Math.round(value);
    }

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
    public Pose2d pose = new Pose2d(-48, 0, Math.toRadians(90)); // Robot's true pose
    public PoseVelocity2d poseVelocity = new PoseVelocity2d(new Vector2d(40, 40), Math.toRadians(400)); // Robot's true pose velocity
    public Dimension robotSize = new Dimension(24, 18); // Size in inches of user's robot
    public DashboardCanvas canvas; // Canvas for the entire window frame

    private Field field;
    private MecanumDrive.Params kinematics; // Kinematic parameters for the simulation
    private PoseVelocity2d requestedVelocity; // Velocity requested by MecanumDrive

    public Simulation() {
        DashboardWindow dashboardWindow = new DashboardWindow("FTC Dashboard", 1280, 720);
        dashboardWindow.setVisible(true);
        canvas = dashboardWindow.getCanvas();
        field = new Field(this);
    }

    // Set the kinematics for the simulation to use:
    public void setKinematics(MecanumDrive.Params kinematics) {
        this.kinematics = kinematics;
    }

    // Update the simulation's velocity to accommodate the request:
    public void requestVelocity(PoseVelocity2d velocity) { // Field relative, inch/s and radian/s
        this.requestedVelocity = velocity;
    }

    // Move the robot in the requested direction via kinematics:
    public void advance(double dt) {
        // Maintain the current velocity if no-one has called requestVelocity yet:
        if (requestedVelocity == null)
            requestedVelocity = poseVelocity;

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

        double requestedAngle = Math.atan2(requestedLinearY, requestedLinearX);
        double currentAngle = Math.atan2(currentLinearY, currentLinearX); // Rise over run
        double theta = requestedAngle - currentAngle; // Angle from current to requested
        double currentVelocity = Math.hypot(currentLinearX, currentLinearY);
        double requestedVelocity = Math.hypot(requestedLinearX, requestedLinearY);
        double currentRadialVelocity = Math.cos(theta) * currentVelocity;
        double currentTangentVelocity = Math.sin(theta) * currentVelocity;
        double deltaRadial = requestedVelocity - currentRadialVelocity;
        if (deltaRadial >= 0) { // Increase radial velocity
            currentRadialVelocity += kinematics.maxProfileAccel * dt;
            currentRadialVelocity = Math.min(currentRadialVelocity, kinematics.maxWheelVel);
            currentRadialVelocity = Math.min(currentRadialVelocity, requestedVelocity);
        } else { // Decrease radial velocity
            currentRadialVelocity += kinematics.minProfileAccel * dt; // minProfileAccel is negative
            currentRadialVelocity = Math.max(currentRadialVelocity, -kinematics.maxWheelVel);
            currentRadialVelocity = Math.max(currentRadialVelocity, requestedVelocity);
        }
        // Drive the tangential velocity to zero:
        if (currentTangentVelocity >= 0) {
            currentTangentVelocity += kinematics.minProfileAccel * dt; // minProfileAccel is negative
            currentTangentVelocity = Math.max(currentTangentVelocity, 0);
        } else {
            currentTangentVelocity += kinematics.maxProfileAccel * dt;
            currentTangentVelocity = Math.min(currentTangentVelocity, 0);
        }
        currentLinearX = Math.cos(currentAngle) * currentRadialVelocity
                       + Math.cos(currentAngle + Math.PI / 2) * currentTangentVelocity;
        currentLinearY = Math.sin(currentAngle) * currentRadialVelocity
                       + Math.sin(currentAngle + Math.PI / 2) * currentTangentVelocity;

        // Update our official pose and velocity:
        pose = new Pose2d(
                pose.position.x + dt * currentLinearX,
                pose.position.y + dt * currentLinearY,
                pose.heading.log() + dt * currentAngular);
        poseVelocity = new PoseVelocity2d(new Vector2d(currentLinearX, currentLinearY), currentAngular);
    }

    public void update(double deltaT) {
        advance(deltaT);

        // All Graphics objects can be cast to Graphics2D:
        Graphics2D g = (Graphics2D) canvas.getBufferStrategy().getDrawGraphics();

        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        field.render(g);

        g.dispose();
        canvas.getBufferStrategy().show();
    }
}
