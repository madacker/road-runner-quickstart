package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * Management and UI for settings.
 */
public class Settings {
    private static Settings settings; // Points to our own  singleton object
    Telemetry telemetry; // Telemetry object used for output
    Gamepad gamepad; // Gamepad to use for settings control
    boolean menuEnabled = false; // True if the setting menu is currently up
    ArrayList<Option> options = new ArrayList<>(); // List of options in the settings menu
    int current; // Option with the UI focus

    abstract private static class Option {
        abstract public String string();
    }
    private static class ToggleOption extends Option {
        String description;
        boolean value;
        Consumer<Boolean> callback;

        public ToggleOption(String description, boolean value, Consumer<Boolean> callback) {
            this.description = description; this.value = value; this.callback = callback;
        }
        public String string() {
            // From https://www.alt-codes.net/circle-symbols // "✅" : "❌";
            return (value ? "●" : "○") + " " + description;
        }
    }
    private static class ListOption extends Option {
        String description;
        int index;
        String[] list;
        Consumer<Integer> callback;

        public ListOption(String description, int index, String[] list, Consumer<Integer> callback) {
            this.description = description; this.index = index; this.list = list; this.callback = callback;
        }
        public String string() {
            return description + ": <b>" + list[index] + "</b>";
        }
    }

    // Button press state:
    private boolean[] buttonPressed = new boolean[7];
    private boolean buttonPress(boolean pressed, int index) {
        boolean press = pressed && !buttonPressed[index];
        buttonPressed[index] = pressed;
        return press;
    }

    // Button press status:
    boolean select() { return buttonPress(gamepad.a, 0); }
    boolean cancel() { return buttonPress(gamepad.b, 1); }
    boolean up() { return buttonPress(gamepad.dpad_up, 2); }
    boolean down() { return buttonPress(gamepad.dpad_down, 3); }
    boolean left() { return buttonPress(gamepad.dpad_left, 4); }
    boolean right() { return buttonPress(gamepad.dpad_right, 5); }
    boolean start() { return buttonPress(gamepad.start, 6); }

    // Constructor:
    Settings(Telemetry telemetry, Gamepad gamepad) {
        settings = this;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
    }

    // Update loop for Settings. If true is returned, the caller should not use gamepad input
    // because the Settings UI is active:
    Gamepad update() {
        menuEnabled = (menuEnabled && !start()) || (!menuEnabled && start());
        if (!menuEnabled)
            return gamepad; // ====>

        if (up()) {
            current--;
            if (current < 0)
                current = 0;
        }
        if (down()) {
            current++;
            if (current == options.size())
                current = options.size() - 1;
        }
        StringBuilder output = new StringBuilder();
        output.append("<p2><b>Settings</b></p2>");
        for (int i = 0; i < options.size(); i++) {
            output.append(i == current ? "➤" : " ").append(options.get(i).string()).append("\n");
        }
        output.append("<hr>\n\n"); // @@@ Did this work?
        telemetry.addLine(output.toString());

        Option option = options.get(current);
        if (option instanceof ToggleOption) {
            ToggleOption toggleOption = (ToggleOption) option;
            if (select()) {
                toggleOption.value = !toggleOption.value;
                toggleOption.callback.accept(toggleOption.value);
            }
        } else if (option instanceof ListOption) {
            ListOption listOption = (ListOption) option;
            if (left() || right()) {
                if (left()) {
                    listOption.index--;
                    if (listOption.index < 0)
                        listOption.index = 0;
                }
                if (right()) {
                    listOption.index++;
                    if (listOption.index > listOption.list.length)
                        listOption.index = listOption.list.length - 1;
                }
                listOption.callback.accept(listOption.index);
            }
        }
        return null;
    }

    // Add a toggleable option to the Settings menu:
    public static void addToggle(String description, boolean initialValue, Consumer<Boolean> callback) {
        callback.accept(initialValue);
        settings.options.add(new ToggleOption(description, initialValue, callback));
    }
    // Add a list option to the Settings menu:
    public static void addList(String description, int initialIndex, String[] list, Consumer<Integer> callback) {
        callback.accept(initialIndex);
        settings.options.add(new ListOption(description, initialIndex, list, callback));
    }
}
