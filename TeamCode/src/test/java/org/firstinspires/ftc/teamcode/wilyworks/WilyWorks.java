package org.firstinspires.ftc.teamcode.wilyworks;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
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
    public void runOpMode(String opModeClassName) {
        // Create our own Class Loader to watch class loads:
        WilyClassLoader loader = new WilyClassLoader();

        // Invoke the specified opMode:
        try {
            Class<?> clazz = loader.loadClass(opModeClassName);
            clazz.getMethod("runOpMode").invoke(clazz.newInstance());
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException |
                 InstantiationException | ClassNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}
