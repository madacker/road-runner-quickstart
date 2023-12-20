package com.example.uitesting;

import com.example.uitesting.ui.WindowFrame;

import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import java.util.Scanner;

class KeyDispatcher implements KeyEventDispatcher {
    private Gamepad gamepad;
    public KeyDispatcher(Gamepad gamepad) {
        this.gamepad = gamepad;
    }
    @Override
    public boolean dispatchKeyEvent(KeyEvent keyEvent) {
        int code = keyEvent.getKeyCode();
        boolean isPressed = (keyEvent.getID() != KeyEvent.KEY_RELEASED);
        System.out.println(String.format("ID: %d", keyEvent.getID()));

        switch (code) {
            case KeyEvent.VK_UP: gamepad.dpad_up = isPressed; break;
            case KeyEvent.VK_DOWN: gamepad.dpad_down = isPressed; break;
            case KeyEvent.VK_A: gamepad.a = isPressed; break;
            case KeyEvent.VK_B: gamepad.b = isPressed; break;
            case KeyEvent.VK_X: gamepad.x = isPressed; break;
            case KeyEvent.VK_Y: gamepad.y = isPressed; break;
        }
        return true;
    }
}

class Gamepad {
    public boolean dpad_down;
    public boolean dpad_up;
    public boolean a;
    public boolean b;
    public boolean x;
    public boolean y;

    Gamepad() {
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyDispatcher(this));
    }
}

class Telemetry {
    WindowFrame windowFrame;
    Canvas canvas;
    ArrayList<String> lineList = new ArrayList<>();

    Telemetry() {
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
    void addLine(String string) {
        lineList.add(string);
    }
    void update() {
        int FONT_SIZE = 14;
        canvas = windowFrame.getCanvas();
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        g.setFont(new Font("Sans", Font.BOLD, FONT_SIZE));

        int x = 100;
        int y = 100;
        for (String line: lineList) {
            g.drawString(line, x, y);
            y += FONT_SIZE;
        }
        g.dispose();
        canvas.getBufferStrategy().show();

        // Erase all of the lines in the list:
        lineList.clear();
    }
}

public class UiTesting {
    public static void main(String[] args) {
        Telemetry telemetry = new Telemetry();
        Gamepad gamepad = new Gamepad();

        while (true) {
            if (gamepad.a) {
                telemetry.addLine("A pressed");
            } else {
                telemetry.addLine("A inactive");
            }
            if (gamepad.b) {
                telemetry.addLine("B pressed");
            } else {
                telemetry.addLine("B inactive");
            }
            telemetry.update();
        }
    }
}