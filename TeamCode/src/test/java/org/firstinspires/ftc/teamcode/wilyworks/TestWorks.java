package org.firstinspires.ftc.teamcode.wilyworks;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;

/**
 * Class loader to catch problematic classes in unit tests and suggest fixes.
 */
class TestClassLoader extends ClassLoader {
    @Override
    public Class<?> loadClass(String name) throws ClassNotFoundException {
        if (true) return super.loadClass(name); // @@@

        // Don't bother scanning system classes:
        if (name.startsWith("java.") || name.startsWith("android.")) {
            return super.loadClass(name);
        }

        System.out.printf("> WilyClassLoader '%s'\n", name);

        // Load the class via the system loader and `get an input stream to its contents:
        Class<?> klass = super.loadClass(name);
        String classAsPath = klass.getName().replace('.', '/') + ".class";
        InputStream stream = super.getResourceAsStream(classAsPath);

        // Read the stream into a byte array:
        byte[] bytes;
        try {
            bytes = new byte[stream.available()];
            DataInputStream in = new DataInputStream(stream);
            in.readFully(bytes);
            in.close();
        } catch (IOException e) {
            throw new RuntimeException("WilyClassLoader I/O failed");
        }

        // Load the result:
        klass = defineClass(name, bytes, 0, bytes.length);
        resolveClass(klass);
        return klass;
    }
}

/**
 * Master Wily Works class.
 */
public class TestWorks {
    // Look for a field in the class and all of its superclasses:
    public static Field findField(Class<?> klass, String fieldName) throws NoSuchFieldException {
        while (true) {
            try {
                Field field = klass.getDeclaredField(fieldName);
                field.setAccessible(true);
                return field; // ===>
            } catch(NoSuchFieldException e) {
                klass = klass.getSuperclass();
                if (klass == null)
                    throw new NoSuchFieldException();
            }
        }
    }

    private void fixup(String opModeClassName) throws ClassNotFoundException, NoSuchMethodException, IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchFieldException {
        // @@@ Test with not-an-opmode
        // Create our own Class Loader to watch class loads:

//        TestClassLoader loader = new TestClassLoader();
//        Class<?> klass = loader.loadClass(opModeClassName);
//        Object instance = klass.newInstance();
//
//        // Apply some fixups:
//        Field telemetryField = findField(klass, "telemetry");
//        Class<?> wilyTelemetryClass = loader.loadClass(WilyTelemetry.class.getName());
//        telemetryField.set(instance, wilyTelemetryClass.newInstance());
//
//        Field hardwareMapField = findField(klass, "hardwareMap");
//        Class<?> hardwareMapClass = loader.loadClass(HardwareMap.class.getName()); // @@@ loader.loadClass("com.qualcomm.robotcore.hardware.HardwareMap");
//        hardwareMapField.set(instance, new TestHardwareMap());
//
//        klass.getMethod("runOpMode").invoke(instance);
    }

    public void runOpMode(String opModeClassName) {
        try {
            fixup(opModeClassName);
        } catch (ClassNotFoundException | NoSuchMethodException | IllegalAccessException |
                 InstantiationException | InvocationTargetException | NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }
}
