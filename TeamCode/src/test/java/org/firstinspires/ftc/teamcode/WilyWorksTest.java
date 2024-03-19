package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import java.lang.reflect.InvocationTargetException;

public class WilyWorksTest {

    @Test
    public void beWily() throws ClassNotFoundException, IllegalAccessException, InstantiationException, NoSuchMethodException, InvocationTargetException {
        WilyClassLoader loader = new WilyClassLoader();

        // Load the OpMode and invoke an instance:
        Class<?> clazz = loader.loadClass("org.firstinspires.ftc.teamcode.explorations.DistanceTest");
        clazz.getMethod("runOpMode").invoke(clazz.newInstance());
    }
}
