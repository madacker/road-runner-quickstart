package org.firstinspires.ftc.teamcode;

import com.sun.tools.javac.resources.javac;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.ArrayList;

/**
 * Our custom implementation of the ClassLoader.
 * For any of classes from "javablogging" package
 * it will use its {@link WilyClassLoader#getClass()}
 * method to load it from the specific .class file. For any
 * other class it will use the super.loadClass() method
 * from ClassLoader, which will eventually pass the
 * request to the parent.
 *
 */
public class WilyClassLoader extends ClassLoader {

    /**
     * Parent ClassLoader passed to this constructor
     * will be used if this ClassLoader can not resolve a
     * particular class.
     *
     * @param parent Parent ClassLoader
     *              (may be from getClass().getClassLoader())
     */
    public WilyClassLoader(ClassLoader parent) {
        super(parent);
        System.out.println("\n\n\n*********************** WilyClassLoader initialized!\n\n\n\n");
    }

    /**
     * Loads a given class from .class file just like
     * the default ClassLoader. This method could be
     * changed to load the class over network from some
     * other server or from the database.
     *
     * @param name Full class name
     */
    private Class<?> getClass(String name) {
        // We are getting a name that looks like
        // javablogging.package.ClassToLoad
        // and we have to convert it into the .class classAsPath name
        // like javablogging/package/ClassToLoad.class

        // @@@  https://frankkieviet.blogspot.com/2009/03/javalanglinkageerror-loader-constraint.html says this:
        // In fact, it's a requirement to delegate all class load requests to the parent classloader for all classes that are in java.* and javax.*.

        String referenceName = name;
        if (name.equals("org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl"))
            referenceName = "org.firstinspires.ftc.teamcode.WilyTelemetryImpl";

        if (true) {
            // Get the system loader to load the class:
            Class<?> klass;
            try {
                klass = getParent().loadClass(referenceName);
            } catch (ClassNotFoundException e) {
                throw new RuntimeException(e);
            }
            String className = klass.getName();
            String classAsPath = className.replace('.', '/') + ".class";
            InputStream stream = klass.getClassLoader().getResourceAsStream(classAsPath);

            // Read the stream into a byte array:
            ByteArrayOutputStream buffer = new ByteArrayOutputStream();
            int nRead;
            byte[] data = new byte[16384];
            try {
                while ((nRead = stream.read(data, 0, data.length)) != -1) {
                    buffer.write(data, 0, nRead);
                }
            } catch (IOException e) {
                return null; // ====>
            }

            // Convert back to a class:
            byte[] bytes = buffer.toByteArray();
            Class<?> c = defineClass(name, bytes, 0, bytes.length);
            resolveClass(c);
            return c;
        } else {
            String classAsPath = name.replace('.', File.separatorChar)
                    + ".class";

            byte[] b = null;
            try {
                // This loads the byte code data from the classAsPath
                b = loadClassData(classAsPath);
                // defineClass is inherited from the ClassLoader class
                // and converts the byte array into a Class
                Class<?> c = defineClass(name, b, 0, b.length);
                resolveClass(c);
                return c;
            } catch (IOException e) {
                e.printStackTrace();
                return null;
            }
        }
    }

    /**
     * Every request for a class passes through this method.
     * If the requested class is in "javablogging" package,
     * it will load it using the
     * {@link WilyClassLoader#getClass()} method.
     * If not, it will use the super.loadClass() method
     * which in turn will pass the request to the parent.
     *
     * @param name
     *            Full class name
     */
    @Override
    public Class<?> loadClass(String name)
            throws ClassNotFoundException {
        if (name.startsWith("org.firstinspires.ftc.") || name.startsWith("com.qualcomm")) {
            System.out.println("loading class '" + name + "'");
            return getClass(name);
        }
        System.out.println("ignoring class '" + name + "'");
        return super.loadClass(name);
    }

//    protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
//        System.out.println("*** Messing up class '" + name + "'" + resolve);
//        return super.loadClass(name, resolve);
//    }

    /**
     * Loads a given file (presumably .class) into a byte array.
     * The file should be accessible as a resource, for example
     * it could be located on the classpath.
     *
     * @param name File name to load
     * @return Byte array read from the file
     * @throws IOException Is thrown when there
     *               was some problem reading the file
     */
    private byte[] loadClassData(String name) throws IOException {
        // Opening the file
        String classPath = "C:\\Source\\Quick10\\TeamCode\\build\\intermediates\\javac\\debug\\classes\\";
        String fullPath = classPath + name;
        InputStream stream = new FileInputStream(new File(fullPath));
//        InputStream stream = this.getClass().getClassLoader().getResourceAsStream(fullPath);
//        InputStream stream = getClass().getClassLoader().getResourceAsStream(name);

        int size = stream.available();
        byte[] buff = new byte[size];
        DataInputStream in = new DataInputStream(stream);
        // Reading the binary data
        in.readFully(buff);
        in.close();
        return buff;
    }
}