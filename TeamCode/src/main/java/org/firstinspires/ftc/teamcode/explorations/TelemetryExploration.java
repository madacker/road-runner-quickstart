package org.firstinspires.ftc.teamcode.explorations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Telemetry Exploration",group="Explore")
public class TelemetryExploration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.speak("Warning, warning!");
        telemetry.addLine("123456789,123456789,123456789,123456789,123456789,123456789,123456789.");
        telemetry.addLine("WWWWWWWWW,WWWWWWWWW,WWWWWWWWW,WWWWWWWWW,WWWWWWWWW,WWWWWWWWW,WWWWWWWWW,");
        for (int i = 2; i < 50; i++) {
            telemetry.addLine(String.format("Line %d", i));
        }
        telemetry.update();

        waitForStart();
    }
}
