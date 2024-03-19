package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import java.lang.reflect.InvocationTargetException;

public class WilyWorksTest {

    @Test
    public void beWily() {
        WilyClassLoader loader = new WilyClassLoader();
        Class<?> clazz;
        Object instance;

        // Load the OpMode:
        try {
            clazz = loader.loadClass("org.firstinspires.ftc.teamcode.explorations.DistanceTest");
        } catch (ClassNotFoundException e) {
            throw new RuntimeException(e);
        }

        // Create an instance of the OpMode:
        try {
            instance = clazz.newInstance();
        } catch (IllegalAccessException|InstantiationException e) {
            throw new RuntimeException(e);
        }

        // Run the OpMode:
        try {
            clazz.getMethod("runOpMode").invoke(instance);
        } catch (InvocationTargetException | IllegalAccessException | NoSuchMethodException e) {
            throw new RuntimeException(e);
        }
    }
}
