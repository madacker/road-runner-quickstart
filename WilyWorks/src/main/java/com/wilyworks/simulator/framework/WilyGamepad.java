package com.wilyworks.simulator.framework;

import com.badlogic.gdx.controllers.Controller;

import org.libsdl.SDL;
import org.libsdl.SDL_Error;

import uk.co.electronstudio.sdl2gdx.SDL2ControllerManager;

public class WilyGamepad {

    public volatile float left_stick_x = 0f;
    public volatile float left_stick_y = 0f;
    public volatile float right_stick_x = 0f;
    public volatile float right_stick_y = 0f;
    public volatile boolean dpad_up = false;
    public volatile boolean dpad_down = false;
    public volatile boolean dpad_left = false;
    public volatile boolean dpad_right = false;
    public volatile boolean a = false;
    public volatile boolean b = false;
    public volatile boolean x = false;
    public volatile boolean y = false;
    public volatile boolean guide = false;
    public volatile boolean start = false;
    public volatile boolean back = false;
    public volatile boolean left_bumper = false;
    public volatile boolean right_bumper = false;
    public volatile boolean left_stick_button = false;
    public volatile boolean right_stick_button = false;
    public volatile float left_trigger = 0f;
    public volatile float right_trigger = 0f;
    public volatile boolean circle = false;
    public volatile boolean cross = false;
    public volatile boolean triangle = false;
    public volatile boolean square = false;
    public volatile boolean share = false;
    public volatile boolean options = false;
    //    public volatile boolean touchpad = false;
//    public volatile boolean touchpad_finger_1;
//    public volatile boolean touchpad_finger_2;
//    public volatile float touchpad_finger_1_x;
//    public volatile float touchpad_finger_1_y;
//    public volatile float touchpad_finger_2_x;
//    public volatile float touchpad_finger_2_y;
    public volatile boolean ps = false;

    SDL2ControllerManager controllerManager;

    public WilyGamepad() {
        controllerManager = new SDL2ControllerManager();
    }

    void updateButtonAliases(){
        // There is no assignment for touchpad because there is no equivalent on XBOX controllers.
        circle = b;
        cross = a;
        triangle = y;
        square = x;
        share = back;
        options = start;
        ps = guide;
    }

    private float deadZone(float value) {
        final double EPSILON = 0.05f;
        if (Math.abs(value) <= EPSILON)
            value = 0;
        return value;
    }

    // Poll the attached game controller to update the button and axis states
    public void update() {
        int count = controllerManager.getControllers().size;
        Controller controller = null;
        if (count != 0) {
            controller = controllerManager.getControllers().get(0);
            try {
                controllerManager.pollState();
            } catch (SDL_Error e) {
                return;
            }

            a = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_A);
            b = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_B);
            x = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_X);
            y = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_Y);
            back = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_BACK);
            guide = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_GUIDE);
            start = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_START);
            dpad_up = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_UP);
            dpad_down = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN);
            dpad_left = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT);
            dpad_right = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
            left_bumper = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
            right_bumper = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
            left_stick_button = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_LEFTSTICK);
            right_stick_button = controller.getButton(SDL.SDL_CONTROLLER_BUTTON_RIGHTSTICK);

            left_stick_x = deadZone(controller.getAxis(SDL.SDL_CONTROLLER_AXIS_LEFTX));
            left_stick_y = deadZone(controller.getAxis(SDL.SDL_CONTROLLER_AXIS_LEFTY));
            right_stick_x = deadZone(controller.getAxis(SDL.SDL_CONTROLLER_AXIS_RIGHTX));
            right_stick_y = deadZone(controller.getAxis(SDL.SDL_CONTROLLER_AXIS_RIGHTY));
            left_trigger = deadZone(controller.getAxis(SDL.SDL_CONTROLLER_AXIS_TRIGGERLEFT));
            right_trigger = deadZone(controller.getAxis(SDL.SDL_CONTROLLER_AXIS_TRIGGERRIGHT));
        }
    }
}
