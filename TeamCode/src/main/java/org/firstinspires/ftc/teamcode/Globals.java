package org.firstinspires.ftc.teamcode;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;

/**
 * {@link Globals} contains all state that can be access globally anytime
 * within the sensor loop.
 */
public class Globals {
    public static Telemetry telemetry;          // Drive Station telemetry
    public static TelemetryPacket packet;       // FTC Dashboard telemetry
    public static Canvas canvas;                // FTC Dashboard telemetry drawing

    private static TimeSplitter splitter;       // Performance timer
    private static Telemetry initialTelemetry;  // Non-null if initialize() has been called

    // Initialize all of our static state just in case we previously crashed:
    public static void initialize(Telemetry telemetry) {
        telemetry = null;
        packet = null;
        canvas = null;
        splitter = TimeSplitter.create("> Loop");
        initialTelemetry = telemetry;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    // Mark the start of the sensor loop.
    public static void startLoop() {
        if (Globals.initialTelemetry == null) {
            throw new IllegalArgumentException("Forgot to call Globals.initialize()");
        }
        if (Globals.packet != null) {
            throw new IllegalArgumentException("Missing Loop.end() call");
        }
        Globals.telemetry = initialTelemetry;
        Globals.packet = new TelemetryPacket();
        Globals.canvas = packet.fieldOverlay();
        Globals.splitter.startSplit();
    }

    // Mark the end of the sensor loop.
    public static void endLoop() {
        if (packet == null) {
            throw new IllegalArgumentException("Missing Loop.start() call");
        }

        splitter.endSplit();
        Globals.telemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry = null;
        packet = null;
        canvas = null;
    }

    // Return a high resolution time count, in seconds:
    public static double time() {
        return nanoTime() * 1e-9;
    }

    // Normalize an angle to [180, -180):
    static double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle <= -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }
}
