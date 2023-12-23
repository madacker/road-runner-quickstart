package com.example.uitesting.ui;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;

/**
 * Windows hook for key presses.
 */
class KeyDispatcher implements KeyEventDispatcher {
    private Gamepad gamepad;
    public KeyDispatcher(Gamepad gamepad) {
        this.gamepad = gamepad;
    }
    @Override
    public boolean dispatchKeyEvent(KeyEvent keyEvent) {
        int code = keyEvent.getKeyCode();
        boolean isPressed = (keyEvent.getID() != KeyEvent.KEY_RELEASED);

        switch (code) {
            case KeyEvent.VK_UP:
                gamepad.dpad_up = isPressed;
                gamepad.left_stick_y = (isPressed) ? 0.9f : 0.0f;
                break;
            case KeyEvent.VK_DOWN:
                gamepad.dpad_down = isPressed;
                gamepad.left_stick_y = (isPressed) ? -0.9f : 0.0f;
                break;
            case KeyEvent.VK_LEFT:
                gamepad.dpad_left = isPressed;
                gamepad.left_stick_x = (isPressed) ? -0.9f : 0.0f;
                break;
            case KeyEvent.VK_RIGHT:
                gamepad.dpad_right = isPressed;
                gamepad.left_stick_x = (isPressed) ? 0.9f : 0.0f;
                break;

            case KeyEvent.VK_A:          gamepad.a = isPressed;    break;
            case KeyEvent.VK_B:          gamepad.b = isPressed;    break;
            case KeyEvent.VK_X:          gamepad.x = isPressed;    break;
            case KeyEvent.VK_Y:          gamepad.y = isPressed;    break;
            case KeyEvent.VK_ENTER:      gamepad.menu = isPressed; break;
            case KeyEvent.VK_BACK_SPACE: gamepad.back = isPressed; break;
        }
        return true;
    }
}

/**
 * This class implements a lightweight emulation of FTC Gamepad that can run on the PC.
 */
public class Gamepad {
    volatile public float left_stick_x;
    volatile public float left_stick_y;
    volatile public boolean dpad_down;
    volatile public boolean dpad_up;
    volatile public boolean dpad_left;
    volatile public boolean dpad_right;
    volatile public boolean a;
    volatile public boolean b;
    volatile public boolean x;
    volatile public boolean y;
    volatile public boolean back;
    volatile public boolean menu;

    public Gamepad() {
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyDispatcher(this));
    }
}
