package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;

import java.util.ArrayList;
import java.util.Comparator;

public class Stats {
    private static double windowStartTime = Globals.time(); // Start time of loop time window
    private static int windowCount = 0; // Count of loops in window
    private static double loopTime = 0; // Average sensor loop time over a second, in seconds

    public static double cameraFps; // Camera frame-rate
    public static double pipelineLatency; // Pipeline latency for the camera, in seconds
    public static String poseStatus = ""; // String describing the last April Tags pose result
    public static double yawCorrection; // How far off Poser() is from the IMU
    public static double imuYaw; // Current yaw as read from the IMU

    // Velocity statistics:
    public static double maxLinearAcceleration = 0;
    public static double minLinearDeceleration = 0;
    public static double maxLinearSpeed = 0;
    public static double previousLinearSpeed = 0;
    public static double maxAngularAcceleration = 0;
    public static double minAngularDeceleration = 0;
    public static double maxAngularSpeed = 0;
    public static double previousAngularSpeed = 0;
    public static double fullAxialSpeed = 0;

    // Track all I/O performance:
    public static ArrayList<TimeSplitter> ioTimes = new ArrayList<>();

    Stats() {
        Settings.registerStats("Stats::Performance", Stats::getPerformance);
        Settings.registerStats("Stats::I/O", Stats::getIo);
    }

    // Call this update every loop iteration:
    static public void update() {
        // Use a 1-second sliding window to track our loop time:
        windowCount++;
        double intervalDuration = Globals.time() - windowStartTime;
        if (intervalDuration > 1.0) {
            loopTime = intervalDuration / windowCount;
            windowStartTime = Globals.time();
            windowCount = 0;
        }
    }

    // Always show this header at the top of the screen:
    static public String getHeader() {
        return String.format("<h6>Loop: %.1fms, Camera FPS: %.1f, Latency: %.1f</h6>",
                loopTime * 1000.0, cameraFps, pipelineLatency * 1000.0);
    }

    // Get a summary of the I/O times:
    static public String getIo() {
        double sumMs = 0;
        double loopMs = Stats.loopTime * 1000.0;

        // Sort in decreasing order:
        ioTimes.sort(Comparator.comparingDouble(x -> loopMs - x.getMs()));

        StringBuilder builder = new StringBuilder();
        builder.append("I/O breakdown:\n");
        for (TimeSplitter time: ioTimes) {
            double ms = time.getMs();
            if (ms > 0) {
                builder.append(String.format("&emsp;%.1f ms: %s\n", ms, time.getDescription()));
                sumMs += ms;
            }
        }
        builder.append(String.format("&emsp;%.1f ms: Unknown\n", loopMs - sumMs));
        builder.append("------------------\n");
        builder.append(String.format("&emsp;%.1f ms: Total loop time", loopMs));

        return builder.toString();
    }

    // Summarize all of the statistics into a nice string:
    @SuppressLint("DefaultLocale")
    static public String getPerformance() {
        String result = "<h1>Stats</h1>";
        result += poseStatus + "\n";

        double degreesCorrection = Math.toDegrees(yawCorrection);
        while (degreesCorrection <= -45)
            degreesCorrection += 90;
        while (degreesCorrection > 45)
            degreesCorrection -= 90;
        result += String.format("Yaw correction: %.2f°, IMU yaw: %.2f°\n", degreesCorrection, Math.toDegrees(imuYaw));

        result += String.format("Linear: top-speed: %.1f, theoretical: %.1f\n    accel: %.1f, decel: %.1f\n",
                maxLinearSpeed, fullAxialSpeed, maxLinearAcceleration, minLinearDeceleration);
        result += String.format("Angular: top-speed: %.2f\n    accel: %.2f, decel: %.2f\n",
                maxAngularSpeed, maxAngularAcceleration, minAngularDeceleration);

        return result;
    }

    // Log interesting velocity data:
    static public void updateVelocity(PoseVelocity2d poseVelocity, double fullAxialSpeed) {
        Stats.fullAxialSpeed = fullAxialSpeed;

        double linearSpeed = Math.hypot(poseVelocity.linearVel.x, poseVelocity.linearVel.y);
        double linearSpeedDelta = linearSpeed - previousLinearSpeed;
        if (linearSpeedDelta > 0) {
            maxLinearAcceleration = Math.max(linearSpeedDelta, maxLinearAcceleration);
        } else {
            minLinearDeceleration = Math.min(linearSpeedDelta, minLinearDeceleration);
        }
        previousLinearSpeed = linearSpeed;
        maxLinearSpeed = Math.max(linearSpeed, maxLinearSpeed);

        double angularSpeed = Math.abs(poseVelocity.angVel);
        double angularSpeedDelta = angularSpeed - previousAngularSpeed;
        if (angularSpeedDelta > 0) {
            maxAngularAcceleration = Math.max(angularSpeedDelta, maxAngularAcceleration);
        } else {
            minAngularDeceleration = Math.min(angularSpeedDelta, minAngularDeceleration);
        }
        previousAngularSpeed = angularSpeed;
        maxAngularSpeed = Math.max(angularSpeed, maxAngularSpeed);
    }
}
