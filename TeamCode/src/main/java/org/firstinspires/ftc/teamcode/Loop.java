package org.firstinspires.ftc.teamcode;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;

public class Loop {
    public static Telemetry telemetry;         // Drive Station telemetry
    public static TelemetryPacket packet;      // FTC Dashboard telemetry
    public static Canvas canvas;               // FTC Dashboard telemetry drawing

    private static TimeSplitter splitter = TimeSplitter.create("> Loop");

    /**
     * Mark the start of the sensor loop.
     */
    public static void start(Telemetry telemetry) {
        if (Loop.packet != null) {
            throw new IllegalArgumentException("Missing Loop.end() call");
        }
        Loop.telemetry = telemetry;
        Loop.packet = new TelemetryPacket();
        Loop.canvas = packet.fieldOverlay();
        Loop.splitter.startSplit();
    }

    /**
     * Mark the end of the sensor loop.
     */
    public static void end() {
        if (packet == null) {
            throw new IllegalArgumentException("Missing Loop.start() call");
        }

        splitter.endSplit();
        Loop.telemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry = null;
        packet = null;
        canvas = null;
    }

    /**
     * Returns a high resolution time count, in seconds.
     */
    public static double time() {
        return nanoTime() * 1e-9;
    }
}
