package com.example.kinematictesting.framework;

import static java.lang.Thread.sleep;

import java.awt.Graphics;

public class Simulation {
    WindowFrame windowFrame;
    MainCanvas canvas;
    Simulation simulation;

    public Simulation() {
        windowFrame = new WindowFrame("UI", 400);
        windowFrame.setVisible(true);
        canvas = windowFrame.getCanvas();
    }

    public void update() {
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
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
