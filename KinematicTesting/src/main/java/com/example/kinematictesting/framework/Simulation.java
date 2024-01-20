package com.example.kinematictesting.framework;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.Pose2d;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsEnvironment;
import java.awt.Image;
import java.awt.Point;
import java.awt.image.BufferedImage;

class Field {
    MainCanvas canvas;
    Point fieldOrigin = new Point(0, 0);
    Dimension fieldSize = new Dimension(720, 720);

    Field(MainCanvas canvas) {
        this.canvas = canvas;
    }
    void render(Pose2d pose) {

    }
}

public class Simulation {
    WindowFrame windowFrame;
    MainCanvas canvas;
    Image backgroundImage;
    BufferedImage robotImage;

    private void redrawBot() {
        GraphicsConfiguration config =
            GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().getDefaultConfiguration();
        // @@@ backgroundImage = config.createCompatibleImage(canvas.
    }

    public Simulation() {
        windowFrame = new WindowFrame("UI", 400);
        windowFrame.setVisible(true);
        canvas = windowFrame.getCanvas();
        backgroundImage = canvas.getBackground("background/season-2023-centerstage/field-2023-juice-dark.png");
    }

    public void update() {
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        g.drawImage(backgroundImage, 0, 0, null);
        g.setColor(new Color(0xffffff));
        g.drawString("Hello kinematic world!", 20, 20);
        g.dispose();
        canvas.getBufferStrategy().show();

        try {
            sleep(20);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
