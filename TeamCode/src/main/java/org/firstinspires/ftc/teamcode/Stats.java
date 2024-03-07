package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;

public class Stats {
    private static HashMap<String, Timer> timers = new HashMap<>(); // All internal timers
    private static ArrayList<Timer> ioTimers = new ArrayList<>(); // I/O timers
    private static double windowStartTime = Globals.time(); // Start of the timing window
    private static int windowLoopCount = 0; // Count of sensor loops during this window
    private static HashMap<String, String> data = new HashMap<>(); // For 'addData()'

    private final static String LOOP_TIMER = "Total loop time"; // Key
    /** @noinspection FieldCanBeLocal*/
    private final double WINDOW_DURATION = 5.0; // Sliding window duration, in seconds

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

    // Class for tracking internal timing:
    static class Timer {
        String descriptor; // Full descriptor including any "::"
        int currentCount; // Count of timings in current window
        double currentSum; // Sum of all timings in current window, in seconds
        double startTime; // Time of the start of the timing bracket; 0 if not in a timing bracket
        double resultTime; // Average of the previous window in seconds
        int resultCount; // Count of invocations in the previous window

        Timer(String descriptor) { this.descriptor = descriptor; }
    }

    Stats() {
        Settings.registerStats("I/O", Stats::getIoSummary);
        Settings.registerStats("Performance", Stats::getPerformanceSummary);
        Settings.registerStats("Telemetry", Stats::getTelemetry);

        startTimer(LOOP_TIMER);
    }

    // Call this update every loop iteration:
    public void update() {
        Stats.endTimer(LOOP_TIMER);
        double windowDuration = Globals.time() - windowStartTime;

        windowLoopCount++;
        if (windowDuration > WINDOW_DURATION) {
            for (Timer timer: Stats.timers.values()) {
                // Compute the per-sensor-loop average, not the per-timer-invocation average:
                timer.resultTime = timer.currentSum / windowLoopCount;
                timer.resultCount = timer.currentCount;
                timer.currentSum = 0;
                timer.currentCount = 0;
            }
            windowStartTime = Globals.time();
            windowLoopCount = 0;
        }
        Stats.startTimer(LOOP_TIMER);
    }

    // Start a timer:
    static public void startTimer(String descriptor) {
        Timer timer = timers.get(descriptor);
        if (timer == null) {
            timer = new Timer(descriptor);
            timers.put(descriptor, timer);
            ioTimers.add(timer);
        }
        assert(timer.startTime == 0);
        timer.startTime = Globals.time();
    }

    // End a timer:
    static public void endTimer(String descriptor) {
        Timer timer = timers.get(descriptor);
        assert(timer != null);
        assert(timer.startTime != 0);
        timer.currentCount++;
        timer.currentSum += Globals.time() - timer.startTime;
        timer.startTime = 0;
    }

    // Always show this header at the top of the screen:
    static public String getHeader() {
        //noinspection DataFlowIssue
        return String.format("<h6>Loop: %.1fms, Camera FPS: %.1f, Latency: %.1f</h6>",
                timers.get(LOOP_TIMER).resultTime * 1000.0, cameraFps, pipelineLatency * 1000.0);
    }

    // Get a summary of the I/O times:
    static public String getIoSummary() {
        double sumMs = 0;
        //noinspection DataFlowIssue
        double loopMs = timers.get(LOOP_TIMER).resultTime * 1000.0;

        // Sort in decreasing order:
        ioTimers.sort(Comparator.comparingDouble(x -> loopMs - x.resultTime));

        StringBuilder builder = new StringBuilder();
        for (Timer timer: ioTimers) {
            double ms = timer.resultTime * 1000.0;
            if ((ms > 0.01) && (timer.descriptor.startsWith("io::"))) {
                builder.append(String.format("&emsp;%.1f ms - %s\n", ms, timer.descriptor));
                sumMs += ms;
            }
        }
        builder.append(String.format("&emsp;%.1f ms - Unaccounted\n", loopMs - sumMs));
        builder.append("------------------\n");
        builder.append(String.format("&emsp;%.1f ms - Total loop time\n", loopMs));

        builder.append("\n");
        for (Timer timer: ioTimers) {
            double ms = timer.resultTime * 1000.0;
            if ((ms > 0.01) && (!timer.descriptor.startsWith("io::")) && !timer.descriptor.equals(LOOP_TIMER)) {
                builder.append(String.format("&emsp;%.1f ms - %s\n", ms, timer.descriptor));
                sumMs += ms;
            }
        }
        return builder.toString();
    }

    // Summarize all of the statistics into a nice string:
    static public String getPerformanceSummary() {
        String result = poseStatus + "\n";

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

    // Get telemetry:
    static public String getTelemetry() {
        StringBuilder builder = new StringBuilder();
        for (String key: data.keySet()) {
            //noinspection StringConcatenationInsideStringBufferAppend
            builder.append(key + ": " + data.get(key));
        }
        return builder.toString();
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

    // Add stats data for our own telemetry and for graphing in FTC Dashboard:
    static public void addData(String caption, Object datum) {
        Globals.packet.put(caption, datum);
        data.put(caption, datum.toString());
    }
}
