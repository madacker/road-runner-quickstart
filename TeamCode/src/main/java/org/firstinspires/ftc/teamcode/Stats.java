package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.PoseVelocity2d;

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
    @SuppressLint("DefaultLocale")
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

        result += String.format("Linear: top-speed: %.1f, theoretical: %.1f, accel: %.1f, decel: %.1f",
                maxLinearSpeed, fullAxialSpeed, maxLinearAcceleration, minLinearDeceleration);
        result += String.format("Angular: top-speed: %.2f, accel: %.2f, decel: %.2f",
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
