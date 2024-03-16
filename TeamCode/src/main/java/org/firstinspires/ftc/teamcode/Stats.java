package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;

public class Stats {
    public static Stats stats; // Point to the global Stats object

    private HashMap<String, Timer> timers = new HashMap<>(); // All internal timers
    private ArrayList<Timer> ioTimers = new ArrayList<>(); // I/O timers
    private double windowStartTime = Globals.time(); // Start of the timing window
    private int windowLoopCount = 0; // Count of sensor loops during this window
    private HashMap<String, String> data = new HashMap<>(); // For 'addData()'

    private static final String LOOP_TIMER = "Total loop time"; // Key
    /** @noinspection FieldCanBeLocal*/
    private static final double WINDOW_DURATION = 5.0; // Sliding window duration, in seconds

    public double cameraFps; // Camera frame-rate
    public double pipelineLatency; // Pipeline latency for the camera, in seconds
    public String poseStatus = ""; // String describing the last April Tags pose result
    public double yawCorrection; // How far off Poser() is from the IMU
    public double imuYaw; // Current yaw as read from the IMU

    // Velocity statistics:
    public double maxLinearAcceleration = 0;
    public double minLinearDeceleration = 0;
    public double maxLinearSpeed = 0;
    public double previousLinearSpeed = 0;
    public double maxAngularAcceleration = 0;
    public double minAngularDeceleration = 0;
    public double maxAngularSpeed = 0;
    public double previousAngularSpeed = 0;
    public double fullAxialSpeed = 0;

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
        stats = this;
        
        Settings.registerStats("I/O", Stats::getIoSummary);
        Settings.registerStats("Performance", Stats::getPerformanceSummary);
        Settings.registerStats("Telemetry", Stats::getTelemetry);

        startTimer(LOOP_TIMER);
    }

    // Call this update every loop iteration:
    public void update() {
        endTimer(LOOP_TIMER);
        double windowDuration = Globals.time() - windowStartTime;

        windowLoopCount++;
        if (windowDuration > WINDOW_DURATION) {
            for (Timer timer: timers.values()) {
                // Compute the per-sensor-loop average, not the per-timer-invocation average:
                timer.resultTime = timer.currentSum / windowLoopCount;
                timer.resultCount = timer.currentCount;
                timer.currentSum = 0;
                timer.currentCount = 0;
            }
            windowStartTime = Globals.time();
            windowLoopCount = 0;
        }
        startTimer(LOOP_TIMER);
    }

    // Start a timer:
    static public void startTimer(String descriptor) {
        Stats stats = Stats.stats;
        Timer timer = stats.timers.get(descriptor);
        if (timer == null) {
            timer = new Timer(descriptor);
            stats.timers.put(descriptor, timer);
            stats.ioTimers.add(timer);
        }
        assert(timer.startTime == 0);
        timer.startTime = Globals.time();
    }

    // End a timer:
    static public void endTimer(String descriptor) {
        Timer timer = Stats.stats.timers.get(descriptor);
        assert(timer != null);
        assert(timer.startTime != 0);
        timer.currentCount++;
        timer.currentSum += Globals.time() - timer.startTime;
        timer.startTime = 0;
    }

    // Always show this header at the top of the screen:
    static public String getHeader() {
        Stats stats = Stats.stats;
        //noinspection DataFlowIssue
        return String.format("<h6>Loop: %.1fms, Camera FPS: %.1f, Latency: %.1f</h6>",
                stats.timers.get(LOOP_TIMER).resultTime * 1000.0, stats.cameraFps, stats.pipelineLatency * 1000.0);
    }

    // Get a summary of the I/O times:
    static public String getIoSummary() {
        Stats stats = Stats.stats;
        double sumMs = 0;
        //noinspection DataFlowIssue
        double loopMs = stats.timers.get(LOOP_TIMER).resultTime * 1000.0;

        // Sort in decreasing order:
        stats.ioTimers.sort(Comparator.comparingDouble(x -> loopMs - x.resultTime));

        StringBuilder builder = new StringBuilder();
        for (Timer timer: stats.ioTimers) {
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
        for (Timer timer: stats.ioTimers) {
            double ms = timer.resultTime * 1000.0;
            if ((ms > 0.01) && (!timer.descriptor.startsWith("io::")) && !timer.descriptor.equals(LOOP_TIMER)) {
                builder.append(String.format("&emsp;%.2f ms - %s\n", ms, timer.descriptor));
                sumMs += ms;
            }
        }
        return builder.toString();
    }

    // Summarize all of the statistics into a nice string:
    static public String getPerformanceSummary() {
        Stats stats = Stats.stats;
        String result = stats.poseStatus + "\n";

        double degreesCorrection = Math.toDegrees(stats.yawCorrection);
        while (degreesCorrection <= -45)
            degreesCorrection += 90;
        while (degreesCorrection > 45)
            degreesCorrection -= 90;
        result += String.format("Yaw correction: %.2f°, IMU yaw: %.2f°\n", degreesCorrection, Math.toDegrees(stats.imuYaw));

        result += String.format("Linear: top-speed: %.1f, theoretical: %.1f\n    accel: %.1f, decel: %.1f\n",
                stats.maxLinearSpeed, stats.fullAxialSpeed, stats.maxLinearAcceleration, stats.minLinearDeceleration);
        result += String.format("Angular: top-speed: %.2f\n    accel: %.2f, decel: %.2f\n",
                stats.maxAngularSpeed, stats.maxAngularAcceleration, stats.minAngularDeceleration);

        return result;
    }

    // Get telemetry:
    static public String getTelemetry() {
        StringBuilder builder = new StringBuilder();
        for (String key: Stats.stats.data.keySet()) {
            //noinspection StringConcatenationInsideStringBufferAppend
            builder.append(key + ": " + Stats.stats.data.get(key));
        }
        return builder.toString();
    }

    // Log interesting velocity data:
    static public void updateVelocity(PoseVelocity2d poseVelocity, double fullAxialSpeed) {
        Stats stats = Stats.stats;
        stats.fullAxialSpeed = fullAxialSpeed;

        double linearSpeed = Math.hypot(poseVelocity.linearVel.x, poseVelocity.linearVel.y);
        double linearSpeedDelta = linearSpeed - stats.previousLinearSpeed;
        if (linearSpeedDelta > 0) {
            stats.maxLinearAcceleration = Math.max(linearSpeedDelta, stats.maxLinearAcceleration);
        } else {
            stats.minLinearDeceleration = Math.min(linearSpeedDelta, stats.minLinearDeceleration);
        }
        stats.previousLinearSpeed = linearSpeed;
        stats.maxLinearSpeed = Math.max(linearSpeed, stats.maxLinearSpeed);

        double angularSpeed = Math.abs(poseVelocity.angVel);
        double angularSpeedDelta = angularSpeed - stats.previousAngularSpeed;
        if (angularSpeedDelta > 0) {
            stats.maxAngularAcceleration = Math.max(angularSpeedDelta, stats.maxAngularAcceleration);
        } else {
            stats.minAngularDeceleration = Math.min(angularSpeedDelta, stats.minAngularDeceleration);
        }
        stats.previousAngularSpeed = angularSpeed;
        stats.maxAngularSpeed = Math.max(angularSpeed, stats.maxAngularSpeed);
    }

    // Add stats data for our own telemetry and for graphing in FTC Dashboard:
    static public void addData(String caption, Object datum) {
        if (Globals.packet != null) // Allow addData() calls during initialization
            Globals.packet.put(caption, datum);
        Stats.stats.data.put(caption, datum.toString() + "\n");
    }
}
