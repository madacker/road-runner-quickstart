package com.example.uitesting;

import com.example.uitesting.ui.WindowFrame;

import java.awt.*;
import java.awt.event.*;
import java.util.Scanner;

import javax.swing.*;

class KeyDispatcher implements KeyEventDispatcher {
    @Override
    public boolean dispatchKeyEvent(KeyEvent keyEvent) {
        int code = keyEvent.getKeyCode();
        if (keyEvent.getID() == KeyEvent.KEY_PRESSED) {
            if (code == KeyEvent.VK_UP) {
                System.out.println("UP");
                return true;
            } else if (code == KeyEvent.VK_DOWN) {
                System.out.println("DOWN");
                return true;
            }
        }
        return false;
    }
}

class Ui {
    WindowFrame windowFrame;
    Canvas canvas;
    Scanner keyboard;

    Ui() {
        windowFrame = new WindowFrame("UI", 800);
        windowFrame.setVisible(true);

        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyDispatcher());
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