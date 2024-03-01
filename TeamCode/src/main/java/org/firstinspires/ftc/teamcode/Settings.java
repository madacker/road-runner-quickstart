package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * Management and UI for settings.
 */
public class Settings {
    static private final String DESCRIPTOR_SEPARATOR = "::";
    private static Settings settings; // Points to our own  singleton object
    Telemetry telemetry; // Telemetry object used for output
    Gamepad gamepad; // Gamepad to use for settings control
    boolean menuEnabled = false; // True if the setting menu is currently up
    ArrayList<MenuOption> menuStack = new ArrayList<>(); // Stack of menus, the last is the current

    abstract private static class Option {
        String description;
        Option(String descriptor) {
            // The description comes after the last separator:
            int lastIndex = descriptor.lastIndexOf(DESCRIPTOR_SEPARATOR);
            if (lastIndex != -1) {
                descriptor = descriptor.substring(lastIndex + DESCRIPTOR_SEPARATOR.length());
            }
            description = descriptor;
        }
        abstract public String string();
    }
    private static class MenuOption extends Option {
        ArrayList<Option> options = new ArrayList<>(); // List of options in this menu
        int current; // Index of Option in options that has the UI focus
        public MenuOption(String descriptor) {
            super(descriptor);
        }
        public String string() {
            return "\uD83D\uDCC1 " + description + "..."; // Folder symbol
        }
    }
    private static class ToggleOption extends Option {
        boolean value;
        Consumer<Boolean> callback;
        public ToggleOption(String descriptor, boolean value, Consumer<Boolean> callback) {
            super(descriptor); this.value = value; this.callback = callback;
        }
        public String string() {
            // From https://www.alt-codes.net/circle-symbols // "✅" : "❌";
            // return (value ? "●" : "○") + " " + description;
            return (value ? "☒" : "☐") + " " + description;
        }
    }
    private static class ListOption extends Option {
        int index;
        String[] list;
        BiConsumer<Integer, String> callback;
        public ListOption(String descriptor, int index, String[] list, BiConsumer<Integer, String> callback) {
            super(descriptor); this.index = index; this.list = list; this.callback = callback;
        }
        public String string() {
            return "↔️ <b>" + list[index] + "</b>: " + description; // "↔⬌ "
        }
    }
    private static class ActivationOption extends Option {
        Function<Boolean, String> callback;
        public ActivationOption(String descriptor, Function<Boolean, String> callback) {
            super(descriptor); this.callback = callback;
        }
        public String string() {
            return "❗ " + callback.apply(false); // ⚡
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
    boolean left() { return buttonPress(gamepad.dpad_left, 2); }
    boolean right() { return buttonPress(gamepad.dpad_right, 3); }
    boolean up() { return buttonPress(gamepad.dpad_up, 4); }
    boolean down() { return buttonPress(gamepad.dpad_down, 5); }
    boolean start() { return buttonPress(gamepad.start, 6); }

    // Constructor:
    Settings(Telemetry telemetry, Gamepad gamepad) {
        Settings.settings = this;

        this.telemetry = telemetry;
        this.gamepad = gamepad;

        // Create the root settings menu:
        menuStack.add(new MenuOption("Settings"));
    }

    // Animate the cursor:
    private String cursor() {
        // Spinner taken from https://stackoverflow.com/questions/2685435/cooler-ascii-spinners:
        final String spinner = "◇◈◆◈";
        final double CYCLE_TIME = 1.5; // Seconds

        double fraction = (Globals.time() % CYCLE_TIME) / CYCLE_TIME;
        int index = (int) (fraction * spinner.length());
        return " " + spinner.charAt(index);
    }

    // Update loop for Settings. If true is returned, the caller should not use gamepad input
    // because the Settings UI is active:
    Gamepad update() {
        menuEnabled = (menuEnabled && !start()) || (!menuEnabled && start());
        if (!menuEnabled)
            return gamepad; // ====>

        // The current menu is always at the bottom:
        MenuOption menu = menuStack.get(menuStack.size() - 1);
        if (up()) {
            menu.current--;
            if (menu.current < 0)
                menu.current = 0;
        }
        if (down()) {
            menu.current++;
            if (menu.current == menu.options.size())
                menu.current = menu.options.size() - 1;
        }

        StringBuilder output = new StringBuilder();

        // Output a header with the submenu names:
        output.append("<h2>");
        for (int i = 0; i < menuStack.size(); i++) {
            if (i > 0)
                output.append("·");
            output.append(menuStack.get(i).description);
        }
        output.append("</h2>");

        // Now output the options:
        for (int i = 0; i < menu.options.size(); i++) {
            if (i == menu.current) {
                String option = menu.options.get(i).string();
                output.append(option.substring(0, 2) + "<font color='#bfdbfe'>" + cursor() + "</font></i>" + option.substring(2) + "</i>\n");
            }
            else
                output.append(menu.options.get(i).string() + "\n");
        }
        output.append(Stats.get());
        telemetry.addLine(output.toString());

        Option option = menu.options.get(menu.current);
        if (cancel()) {
            if (menuStack.size() > 1) {
                // Pop up the menu stack:
                menuStack.remove(menuStack.size() - 1);
            }
        }
        else if (option instanceof ToggleOption) {
            ToggleOption toggleOption = (ToggleOption) option;
            if (select()) {
                toggleOption.value = !toggleOption.value;
                toggleOption.callback.accept(toggleOption.value);
            }
        } else if (option instanceof ListOption) {
            ListOption listOption = (ListOption) option;
            boolean left = left();
            boolean right = right();
            if (left || right) {
                if (left) {
                    listOption.index--;
                    if (listOption.index < 0)
                        listOption.index = 0;
                }
                if (right) {
                    listOption.index++;
                    if (listOption.index >= listOption.list.length)
                        listOption.index = listOption.list.length - 1;
                }
                listOption.callback.accept(listOption.index, listOption.list[listOption.index]);
            }
        } else if (option instanceof ActivationOption) {
            if (select()) {
                ActivationOption activationOption = (ActivationOption) option;
                activationOption.callback.apply(true);
            }
        } else if (option instanceof MenuOption) {
            if (select()) {
                menuStack.add((MenuOption) option);
            }
        }
        return null; // We own the Gamepad, the caller can't have it
    }

    // Add a new option to the appropriate spot in the menu hierarchy:
    private void register(String descriptor, Option newOption) {
        MenuOption menu = menuStack.get(0); // Root menu

        // Peel off the hierarchy which is in the form "Vision::Configuration::Setting":
        while (true) {
            int index = descriptor.indexOf(DESCRIPTOR_SEPARATOR);
            if (index == -1)
                break; // ====>

            // Peel off the first menu name from the descriptor:
            String submenuName = descriptor.substring(0, index);
            descriptor = descriptor.substring(index + DESCRIPTOR_SEPARATOR.length());

            // Find or create the submenu:
            MenuOption submenu = null;
            for (Option option: menu.options) {
                if ((option instanceof MenuOption) && (option.description.equals(submenuName))) {
                    submenu = (MenuOption) option;
                    break;
                }
            }
            if (submenu == null) {
                submenu = new MenuOption(submenuName);
                menu.options.add(submenu);
            }
            // Descend into that submenu:
            menu = submenu;
        }
        menu.options.add(newOption);
    }

    // Add a toggleable option to the Settings menu:
    public static void registerToggleOption(String descriptor, boolean initialValue, Consumer<Boolean> callback) {
        callback.accept(initialValue);
        settings.register(descriptor, new ToggleOption(descriptor, initialValue, callback));
    }
    // Add a list option to the Settings menu:
    public static void registerListOption(String descriptor, String[] list, int initialIndex, BiConsumer<Integer, String> callback) {
        callback.accept(initialIndex, list[initialIndex]);
        settings.register(descriptor, new ListOption(descriptor, initialIndex, list, callback));
    }
    // Add an option that can only be activated:
    public static void registerActivationOption(String descriptor, Function<Boolean, String> callback) {
        callback.apply(true);
        settings.register(descriptor, new ActivationOption(descriptor, callback));
    }
}
