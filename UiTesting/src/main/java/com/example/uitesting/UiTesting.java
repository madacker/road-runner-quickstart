package com.example.uitesting;

import com.example.uitesting.ui.WindowFrame;

import java.awt.*;
import java.awt.datatransfer.StringSelection;
import java.awt.event.*;
import java.awt.image.BufferStrategy;

import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.border.EtchedBorder;

class Ui {
    WindowFrame windowFrame;
    Canvas canvas;

    Ui() {
        windowFrame = new WindowFrame("UI", 800);
        windowFrame.setVisible(true);
    }
    void render() {
        canvas = windowFrame.getCanvas();
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());

        g.setFont(new Font("Sans", Font.BOLD, 14));
        g.drawString("This is a test!", 100, 100);
        g.dispose();
        canvas.getBufferStrategy().show();
    }
}

public class UiTesting {
    public static void main(String[] args) {
        Ui ui = new Ui();
        while (true)
            ui.render();
    }
}