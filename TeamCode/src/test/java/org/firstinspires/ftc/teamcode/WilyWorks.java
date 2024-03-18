package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.explorations.DistanceTest;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.BlockJUnit4ClassRunner;
import org.junit.runners.model.InitializationError;

import java.lang.reflect.InvocationTargetException;

class ClasspathTestRunner extends BlockJUnit4ClassRunner {
    static ClassLoader customClassLoader;

    public ClasspathTestRunner(Class<?> clazz) throws InitializationError {
        super(loadFromCustomClassloader(clazz));
        System.out.println("ClasspathTestRunner!!!!");
    }

    // Loads a class in the custom classloader
    private static Class<?> loadFromCustomClassloader(Class<?> clazz) throws InitializationError {
        try {
            // Only load once to support parallel tests
            if (customClassLoader == null) {
                customClassLoader = new CustomClassLoader();
            }
            return Class.forName(clazz.getName(), true, customClassLoader);
        } catch (ClassNotFoundException e) {
            throw new InitializationError(e);
        }
    }

    public static class CustomClassLoader extends ClassLoader {
        @Override
        public Class<?> loadClass(String name) throws ClassNotFoundException {
            System.out.printf("Wooooooooooooooooooooah!!!\n");
            return super.loadClass(name);
        }
    }
}

public class WilyWorks {
    @Test
    public void Simple() {
        Integer i = new Integer(5);
        String string = "Simple test: ";
        System.out.printf("%s: %d\n", string, i);
        System.out.print("This is my ClassLoader: "
                + WilyWorks.class.getClassLoader());
    }

    @Test
    public void beWily() {
        WilyClassLoader loader = new WilyClassLoader(WilyWorks.class.getClassLoader());
        Class clazz;
        Object instance;

        // Preload stuff:
//        try {
//            loader.loadClass("com.qualcomm.robotcore.eventloop.opmode.OpModeInternal");
//            loader.loadClass("org.firstinspires.ftc.robotcore.external.Telemetry");
//        } catch (ClassNotFoundException e) {
//            throw new RuntimeException(e);
//        }

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
        } catch (InvocationTargetException e) {
            throw new RuntimeException(e);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        } catch (NoSuchMethodException e) {
            throw new RuntimeException(e);
        }
    }
}
