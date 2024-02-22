package org.firstinspires.ftc.teamcode;

public class Stats {
    private static double windowStartTime = Globals.time(); // Start time of loop time window
    private static int windowCount = 0; // Count of loops in window
    private static double loopTime = 0; // Average sensor loop time over a second, in seconds

    public static double cameraFps; // Camera frame-rate
    public static double pipelineLatency; // Pipeline latency for the camera, in seconds
    public static String poseStatus = ""; // String describing the last April Tags pose result
    public static double yawCorrection;
    public static double imuYaw;

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

    // Summarize all of the statistics into a nice string:
    static public String get() {
        String result = "<p1>Stats</p1>";
        result += String.format("Loop time: %.1fms, Camera FPS: %.1f, Latency: %.1f\n",
            loopTime * 1000.0, cameraFps, pipelineLatency * 1000.0);
        result += poseStatus + "\n";

        double degreesCorrection = Math.toDegrees(yawCorrection);
        while (degreesCorrection <= -45)
            degreesCorrection += 90;
        while (degreesCorrection > 45)
            degreesCorrection -= 90;
        result += String.format("Yaw correction: %.2f°, IMU yaw: %.2f°", degreesCorrection, Math.toDegrees(imuYaw));

        return result;
    }
}
