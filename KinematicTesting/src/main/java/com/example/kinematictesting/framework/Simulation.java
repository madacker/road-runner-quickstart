package com.example.kinematictesting.framework;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.canvas.CanvasOp;
import com.acmerobotics.dashboard.canvas.Circle;
import com.acmerobotics.dashboard.canvas.Fill;
import com.acmerobotics.dashboard.canvas.Polygon;
import com.acmerobotics.dashboard.canvas.Polyline;
import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.dashboard.canvas.Stroke;
import com.acmerobotics.dashboard.canvas.StrokeWidth;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsEnvironment;
import java.awt.Image;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Transparency;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.imageio.ImageIO;

import jdk.internal.org.jline.terminal.Size;

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

    double scaleInchesToPixel(double inches) {
        return (inches / 72.0) * FIELD_SURFACE_DIMENSION;
    }
    Vector2d mirrorX(Vector2d vector) {
        return new Vector2d(-vector.x, vector.y);
    }
    Vector2d mirrorY(Vector2d vector) {
        return new Vector2d(vector.x, -vector.y);
    }
    Vector2d fieldCoordsToScreenCoords(Vector2d vector) {
        return mirrorY(vector)
                .plus(new Vector2d(FIELD_SURFACE_DIMENSION / 2.0, FIELD_SURFACE_DIMENSION / 2.0))
                .times(FIELD_SURFACE_DIMENSION / 144.0)
                .plus(new Vector2d(FIELD_INSET, FIELD_INSET));
    }

    void renderRobot(Graphics2D g) {
        AffineTransform graphicsTransform = new AffineTransform(
                FIELD_SURFACE_DIMENSION / 144.0, 0,
                0, -FIELD_SURFACE_DIMENSION / 144.0,
                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET,
                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET);
        g.transform(graphicsTransform);
        g.translate(simulation.pose.position.x, simulation.pose.position.y);

        g.setColor(new Color(0x00ff00));
        g.fillRect(0, 0, 24, 24);

        AffineTransform imageTransform = new AffineTransform();
        imageTransform.scale((double) simulation.robotSize.width / ROBOT_IMAGE_WIDTH,
                (double) simulation.robotSize.height / ROBOT_IMAGE_HEIGHT);
        imageTransform.rotate(Math.toRadians(45));
        imageTransform.translate(-ROBOT_IMAGE_HEIGHT / 2, -ROBOT_IMAGE_HEIGHT / 2);
        g.drawImage(robotImage, imageTransform, null);


//        transform.translate(FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET + simulation.pose.position.x,
//                FIELD_SURFACE_DIMENSION / 2.0 + FIELD_INSET + simulation.pose.position.y);
//        transform.scale(simulation.robotSize.width / (144.0 * ROBOT_IMAGE_HEIGHT) * FIELD_SURFACE_DIMENSION,
//                        simulation.robotSize.height / (144.0 * ROBOT_IMAGE_WIDTH) * FIELD_SURFACE_DIMENSION);
//        transform.rotate(Math.toRadians(45));
//        transform.translate(-ROBOT_IMAGE_HEIGHT / 2, -ROBOT_IMAGE_HEIGHT / 2);
//        g.drawImage(robotImage, transform, null);


//        Pose2d pose = simulation.pose;
//        Vector2d coords = fieldCoordsToScreenCoords(new Vector2d(pose.position.x, pose.position.y));
//
//        transform.translate(coords.x, coords.y);
//        transform.rotate(Math.toRadians(45)); // @@@ pose.heading.log());
//        transform.translate(scaleInchesToPixel(-ROBOT_WIDTH / 2), scaleInchesToPixel(-ROBOT_HEIGHT / 2));
//        transform.scale(scaleInchesToPixel(ROBOT_WIDTH) / FIELD_SURFACE_DIMENSION,
//                        scaleInchesToPixel(ROBOT_HEIGHT) / FIELD_SURFACE_DIMENSION);
    }

    void renderFieldOverlay(Graphics2D g) {
        if (FtcDashboard.fieldOverlay != null) {
            for (CanvasOp op : FtcDashboard.fieldOverlay.getOperations()) {
                if (op instanceof Circle) {
                    Circle circle = (Circle) op;
                } else if (op instanceof Polygon) {
                    Polygon polygon = (Polygon) op;
                } else if (op instanceof Polyline) {
                    Polyline polyline = (Polyline) op;
                } else if (op instanceof Spline) {
                    Spline spline = (Spline) op;
                } else if (op instanceof Stroke) {
                    Stroke stroke = (Stroke) op;
                } else if (op instanceof Fill) {
                    Fill fill = (Fill) op;
                } else if (op instanceof StrokeWidth) {
                    StrokeWidth strokeWidth = (StrokeWidth) op;
                } else {
                    throw new IllegalArgumentException("Unexpected field overlay op");
                }
            }
        }
    }

    // Render the field, the robot, and the field overlay:
    void render(Graphics2D g) {
        g.setClip(FIELD_VIEW.x, FIELD_VIEW.y, FIELD_VIEW.width, FIELD_VIEW.height);

        // Render the field and the robot:
        g.drawImage(backgroundImage, FIELD_VIEW.x + FIELD_INSET, FIELD_VIEW.y + FIELD_INSET, null);
        renderRobot(g);
        renderFieldOverlay(g);

        // Draw some debug text:
        g.setColor(new Color(0xffffff));
        g.drawString("Hello kinematic world!", 20, 20);
    }
}

public class Simulation {
    public Pose2d pose = new Pose2d(24, 24, 0); // Robot's true pose
    public Dimension robotSize = new Dimension(18, 18); // Size in inches of user's robot
    public MainCanvas canvas; // Canvas for the entire window frame

    private WindowFrame windowFrame;
    private Field field;

    public Simulation() {
        windowFrame = new WindowFrame("UI", 1280, 720);
        windowFrame.setVisible(true);
        canvas = windowFrame.getCanvas();
        field = new Field(this);
    }

    public void update() {
        // All Graphics objects can be cast to Graphics2D:
        Graphics2D g = (Graphics2D) canvas.getBufferStrategy().getDrawGraphics();

        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        field.render(g);

        g.dispose();
        canvas.getBufferStrategy().show();

        try {
            sleep(20);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
