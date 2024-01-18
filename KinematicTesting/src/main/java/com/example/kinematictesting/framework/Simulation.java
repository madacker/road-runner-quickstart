package com.example.kinematictesting.framework;

import static java.lang.Thread.sleep;

import java.awt.Graphics;
import java.awt.Image;

public class Simulation {
    WindowFrame windowFrame;
    MainCanvas canvas;
    Simulation simulation;
    Image backgroundImage;

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
