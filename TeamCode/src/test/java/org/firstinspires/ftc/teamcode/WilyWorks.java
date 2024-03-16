package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.explorations.DistanceTest;
import org.junit.Test;

class WilyClassLoader extends ClassLoader {
    public Class<?> loadClass(String name) throws ClassNotFoundException {
        System.out.printf("*** Loading class name: %s\n", name);
        return super.loadClass(name);
    }
}

public class WilyWorks {
    @Test
    public void beWily() {
        WilyClassLoader loader = new WilyClassLoader();
        Class clazz;
        DistanceTest distanceTest;

        // Load the OpMode:
        try {
            clazz = loader.loadClass("org.firstinspires.ftc.teamcode.explorations.DistanceTest");
        } catch (ClassNotFoundException e) {
            throw new RuntimeException(e);
        }

        // Create an instance of the OpMode:
        try {
            distanceTest = (DistanceTest) clazz.newInstance();
        } catch (IllegalAccessException|InstantiationException e) {
            throw new RuntimeException(e);
        }

        // Run the OpMode:
        try {
            distanceTest.runOpMode();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
