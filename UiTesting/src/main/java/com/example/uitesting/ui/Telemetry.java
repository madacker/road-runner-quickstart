package com.example.uitesting.ui;

import com.example.uitesting.ui.WindowFrame;

import java.awt.Canvas;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.util.ArrayList;

/**
 * This class implements a lightweight emulation of FTC Telemetry that can run on the PC.
 */
public class Telemetry {
    final int FONT_SIZE = 16;
    final Font FONT = new Font("Verdana", Font.PLAIN, FONT_SIZE); // "Sans"
    final int WIDTH_IN_PIXELS = 200;
    final int HEIGHT_IN_LINES = 18;

    WindowFrame windowFrame;
    Canvas canvas;
    ArrayList<String> lineList = new ArrayList<>();

    public Telemetry() {
        windowFrame = new WindowFrame("UI", 800);
        windowFrame.setVisible(true);
    }

    public void addLine(String string) {
        lineList.add(string);
    }

    public void update() {
        canvas = windowFrame.getCanvas();
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        g.setFont(FONT);

        FontMetrics metrics = g.getFontMetrics(FONT);
        System.out.println(String.format("%d, %d\n",
                metrics.stringWidth("123456789,123456789,123456789,123456789,1"),
                metrics.stringWidth("WWWWWWWWW,WWWWWWWWW,WWWWWWW")));
        lineList.add("123456789,123456789,123456789,123456789,1");
        lineList.add("WWWWWWWWW,WWWWWWWWW,WWWWWWW");

        int x = 100;
        int y = 100;
        for (String line : lineList) {
            g.drawString(line, x, y);
            y += FONT_SIZE;
        }
        g.dispose();
        canvas.getBufferStrategy().show();
        lineList.clear();
    }
}
