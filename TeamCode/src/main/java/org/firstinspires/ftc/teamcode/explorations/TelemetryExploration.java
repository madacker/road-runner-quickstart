package org.firstinspires.ftc.teamcode.explorations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Telemetry Exploration",group="Explore")
public class TelemetryExploration extends LinearOpMode {
    private void proportionalTest(Telemetry telemetry) {
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

    private void htmlTest(Telemetry telemetry) {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addLine("This is <b>bold</b>, this is <i>italics</i>, this is <u>underline</u>.");
        telemetry.addLine("<font color='#00ff00'>Green</font> ");
        telemetry.addLine("<h1>The</h1> <h2>quick</h2> <h3>brown</h3> <h4>fox</h4>");

        // Failures:

//        telemetry.addLine("<title>Title!</title>");
//        telemetry.addLine("<div style='color:#d0d0d0;'>Blargh</div>");
//        telemetry.addLine("<div style=\"color:#d0d0d0; font-size: 200%;\">Blargh</div>");
//        telemetry.addLine("<h1 style='background-color:#00ff00;'>My First Heading</h1><p>My first paragraph</p>");
//        telemetry.addLine("<h1 style='color:#00ff00;'>My First Heading</h1><p>My first paragraph</p>");
//        telemetry.addLine("<table style=\"width:100%\">\n" +
//                "  <tr>\n" +
//                "    <td>Emil</td>\n" +
//                "    <td>Tobias</td>\n" +
//                "    <td>Linus</td>\n" +
//                "  </tr>\n" +
//                "</table>");

        telemetry.addLine("<font color='#00ff00'><h1>My First Heading</h1><p>My first paragraph</p>");
        telemetry.addLine("a < b > c & d");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        htmlTest(telemetry);

        telemetry.update();
        waitForStart();
    }
}
