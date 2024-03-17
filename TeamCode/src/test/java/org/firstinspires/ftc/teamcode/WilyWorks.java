package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.explorations.DistanceTest;
import org.junit.Test;

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
