package org.firstinspires.ftc.teamcode;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;

/**
 * Class loader to catch problematic classes in unit tests and suggest fixes.
 */
public class WilyClassLoader extends ClassLoader {
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