package org.firstinspires.ftc.teamcode.wilyworks;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;

/**
 * Class loader to catch problematic classes in unit tests and suggest fixes.
 */
class WilyClassLoader extends ClassLoader {
    @Override
    public Class<?> loadClass(String name) throws ClassNotFoundException {
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
public class WilyWorks {
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

        WilyClassLoader loader = new WilyClassLoader();
        Class<?> klass = loader.loadClass(opModeClassName);
        Object instance = klass.newInstance();

        // Apply some fixups:
        Field telemetryField = findField(klass, "telemetry");
        telemetryField.set(instance, null); // new WilyTelemetry());

        klass.getMethod("runOpMode").invoke(instance);
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
