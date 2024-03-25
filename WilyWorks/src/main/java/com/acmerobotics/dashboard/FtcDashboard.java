package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.TreeMap;

public class FtcDashboard {
    static public TreeMap<String, String> data = new TreeMap<>();
    static public ArrayList<String> log = new ArrayList<>();
    static public Canvas fieldOverlay;
    static FtcDashboard instance = new FtcDashboard();

    public static FtcDashboard getInstance() { return instance; }

    public Telemetry getTelemetry() {
        return null; // ###
    }

    public void sendTelemetryPacket(TelemetryPacket telemetryPacket) {
        // https://www.baeldung.com/java-merge-maps
        telemetryPacket.data.forEach((key, value) -> data.put(key, value));
        log = telemetryPacket.log;
        fieldOverlay = telemetryPacket.fieldOverlay();
    }

    public void withConfigRoot(CustomVariableConsumer function) {
    }
}
