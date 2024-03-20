package com.example.wilyworks.framework;

import com.example.kinematictesting.framework.TelemetryWindow;

import java.awt.Canvas;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.util.ArrayList;

/**
 * This class implements a lightweight emulation of FTC Telemetry that can run on the PC.
 */
public class Telemetry {
    // Enable unit test:
    final boolean TEST = false;

    // Use this font for display on the PC. It's different from the sizing font because the
    // sizing font doesn't support the full unicode character set (like emojis):
    final int DISPLAY_FONT_SIZE = 16;
    final Font DISPLAY_FONT = new Font(null, Font.PLAIN, DISPLAY_FONT_SIZE);

    // Try to emulate the same line width as the REV Driver Station in its horizontal
    // configuration. Because the DS uses a proportional font, and because we don't have
    // access to the source code, we can't precisely replicate the DS behavior when a
    // single line is too wide for the DS and has to be broken into multiple lines. So
    // we use a different proportional font for measuring the text and just kind of guess.
    // We try to use a font that supplies similar proportions for different strings,
    // settling on the following and measuring its width using the strings
    // "123456789,123456789,123456789,123456789,1" and "WWWWWWWWW,WWWWWWWWW,WWWWWWW" which
    // are each as wide as can be displayed on the REV Control Hub without wrapping:
    final Font SIZING_FONT = new Font(null, Font.PLAIN, 10);
    final int WIDTH_IN_FONT_UNITS = 239;
    final int HEIGHT_IN_LINES = 18;

    // Global state:
    public static Telemetry telemetry;

    // Class state:
    TelemetryWindow telemetryWindow;
    Canvas canvas;
    FontMetrics metrics;
    ArrayList<String> lineList = new ArrayList<>();

    // Unit test:
    private void test(Telemetry telemetry) {
        telemetry.addLine("This\uD83C\uDF85\uD83C\uDFFEhas\uD83D\uDD25emojis\uD83C\uDF1Ebetween\u2744\uFE0Fevery\uD83D\uDC14word");
        String emojis = ">";
        for (int i = 0; i < 30; i++) {
            emojis += (true) ? "\uD83C\uDF1E" : "\u2744\uFE0F"; // Surrogate vs. variation selector
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

    // Return the width of the string with all trailing spaces removed:
    private int stringWidth(String string) {
        return metrics.stringWidth(string.replaceAll("\\s+$", ""));
    }

    // PC constructor for a Telemetry object:
    public Telemetry() {
        telemetry = this;

        telemetryWindow = new TelemetryWindow("Telemetry", 400);
        telemetryWindow.setVisible(true);

        canvas = telemetryWindow.getCanvas();
        metrics = canvas.getBufferStrategy().getDrawGraphics().getFontMetrics(SIZING_FONT);
    }

    public void addLine(String string) {
        if (lineList.size() <= HEIGHT_IN_LINES) {
            int newLineIndex;
            while ((newLineIndex = string.indexOf("\n")) != -1) {
                String line = string.substring(0, newLineIndex);
                lineList.add(line);
                string = string.substring(newLineIndex + 1);
            }
            lineList.add(string);
        }
    }

    public void addData(String caption, Object value) {
        addLine(String.format("%s : %s", caption, value.toString()));
    }

    public void addData(String caption, String format, Object... args) {
        addData(caption, String.format(format, args));
    }

    public void addLine() { addLine(""); }
    public void clear() { lineList.clear(); }
    public void clearAll() { lineList.clear(); }

    public void update() {
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        g.setFont(DISPLAY_FONT);

        if (TEST) {
            System.out.println(stringWidth("123456789,123456789,123456789,123456789,12"));
            System.out.println(stringWidth("WWWWWWWWW,WWWWWWWWW,WWWWWWWW"));
            test(this);
        }

        int y = HEIGHT_IN_LINES;
        int lineCount = 0;
        for (String line : lineList) {
            while (lineCount < HEIGHT_IN_LINES) {
                int lineBreak = line.length();
                if (stringWidth(line) > WIDTH_IN_FONT_UNITS) {
                    // If the line is too long, try and break at a space:
                    lineBreak = line.length() - 1;
                    while ((lineBreak > 0) &&
                           ((line.charAt(lineBreak - 1) != ' ') || // -1 to avoid space on next line
                            (stringWidth(line.substring(0, lineBreak - 1)) > WIDTH_IN_FONT_UNITS)))
                        lineBreak--;

                    // If no line break was found using a space, simply break at any character
                    // that isn't a UTF-16 trailing surrogate (the second half of a Unicode
                    // surrogate pair) or a variation selector:
                    if (lineBreak == 0) {
                        lineBreak = line.length() - 1;
                        while (lineBreak > 0) {
                            int character = line.charAt(lineBreak);
                            if (((character & 0xfc00) != 0xdc00) &&
                                    (character != 0xfe0e) &&
                                    (character != 0xfe0f) &&
                                    (stringWidth(line.substring(0, lineBreak)) <= WIDTH_IN_FONT_UNITS))
                                break; // ====>
                            lineBreak--;
                        }
                    }
                }

                g.drawString(line.substring(0, lineBreak), 0, y);
                y += DISPLAY_FONT_SIZE;
                lineCount++;
                line = line.substring(lineBreak);
                if (line.equals(""))
                    break; // ====>
            }
        }
        g.dispose();
        canvas.getBufferStrategy().show();
        lineList.clear();
    }
}
