package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WilyTelemetryImpl {
    WilyTelemetryImpl(OpMode opMode) {
        System.out.println("WilyTelemetryImpl constructor");
    }

    public void setDisplayFormat(Telemetry.DisplayFormat displayFormat) {
        System.out.println("Wily setDisplayFormat!");
    }
}
