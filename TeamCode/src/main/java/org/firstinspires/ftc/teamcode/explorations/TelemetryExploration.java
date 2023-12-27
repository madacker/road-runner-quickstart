package org.firstinspires.ftc.teamcode.explorations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Telemetry Exploration",group="Explore")
public class TelemetryExploration extends LinearOpMode {
    public void test(Telemetry telemetry) {
        telemetry.addLine("This\uD83C\uDF85\uD83C\uDFFEhas\uD83D\uDD25emojis\uD83C\uDF1Ebetween\u2744\uFE0Fevery\uD83D\uDC14word");
        String emojis = ">";
        for (int i = 0; i < 30; i++) {
            emojis += "\uD83C\uDF1E"; // "\u2744\uFE0F" uses an overlay
        }
        telemetry.addLine(emojis);
        telemetry.addLine("This is\nmultiple\nlines followed by an empty line");
        telemetry.addLine("");
        telemetry.addData("Value", 123.0);
        telemetry.addLine("The quick brown fox jumps over the lazy dog. Now is the time for all good men to come to the aid of their party.");
        telemetry.addLine("123456789,123456789,123456789,123456789,12");
        telemetry.addLine("WWWWWWWWW,WWWWWWWWW,WWWWWWWW");
        for (int i = 0; i < 40; i++) {
            telemetry.addLine(String.format("Line %d", i));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        test(telemetry);
        telemetry.update();

        waitForStart();
    }
}
