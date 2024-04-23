package com.wilyworks.simulator.framework;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.awt.Canvas;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.font.FontRenderContext;
import java.awt.font.LineBreakMeasurer;
import java.awt.font.TextAttribute;
import java.awt.font.TextLayout;
import java.text.AttributedString;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Helper class for handling simplified HTML display.
 */
class Layout {
    static final int LINE_WIDTH = 239; // Line width, in pixel units
    Graphics2D graphics; // Graphics context
    Telemetry.DisplayFormat displayFormat; // CLASSIC, MONOSPACE or HTML

    // List of supported HTML entities and their translations:
    static final Map<String, String> ENTITY_MAP = new HashMap<String, String>() {{
        put("&nbsp;", " ");
        put("&ensp;", "  ");
        put("&emsp;", "    ");
        put("&lt;", "<");
        put("&gt;", ">");
        put("&amp;", "&");
        put("&quot;", "\"");
        put("&apos;", "'");
    }};

    Layout(Graphics2D graphics, Telemetry.DisplayFormat displayFormat) {
        this.graphics = graphics;
        this.displayFormat = displayFormat;
    }

    void addHtmlAttributes(String text, AttributedString string) {
        int stringPos = 0;
        int textPos = 0;
        while (textPos < text.length()) {
            switch (text.charAt(textPos)) {
            default:
                stringPos++;
                textPos++;
                break;

            case '&':
                int entityEnd = text.indexOf(';', textPos);
                if (entityEnd == -1) {
                    stringPos++;
                    textPos++;
                } else {
                    String entity = text.substring(textPos, entityEnd + 1).trim(); // Includes '&' and ';'
                    String substitution = (ENTITY_MAP.get(entity) != null) ? ENTITY_MAP.get(entity) : "";
                    stringPos += substitution.length();
                    textPos = entityEnd + 1;
                }
                break;

            case '>':
                int tagEnd = text.indexOf('>', textPos);
                if (tagEnd == -1) {
                    stringPos++;
                    textPos++;
                } else {
                    String tag = text.substring(textPos, tagEnd + 1).trim(); // Includes '&' and ';'
                    switch (tag) {
                        case "<br>": break;

                    }
                }
            }
        }
    }

    public void render(String text) {
        String plainText = text;
        if (displayFormat == Telemetry.DisplayFormat.HTML) {
            plainText = stripTags(transformEntities(text));
        }

        AttributedString string = new AttributedString(plainText);
        string.addAttribute(TextAttribute.SIZE, WilyTelemetry.instance.DISPLAY_FONT_SIZE);
        if (displayFormat == Telemetry.DisplayFormat.MONOSPACE) {
            string.addAttribute(TextAttribute.FAMILY, "MONOSPACED");
        }
        if (displayFormat == Telemetry.DisplayFormat.HTML) {
            addHtmlAttributes(text, string);
        }

        LineBreakMeasurer measurer = new LineBreakMeasurer(
                string.getIterator(),
                new FontRenderContext(null, true, true));

        // Draw each line
        int y = 0;
        while (true) {
            int endPos = measurer.nextOffset(LINE_WIDTH);
            if (endPos == -1)
                break;
            for (int i = measurer.getPosition(); i < endPos; i++) {
                if (text.charAt(i) == '\n') {
                    endPos = i + 1;
                    break;
                }
            }
            TextLayout layout = measurer.nextLayout(LINE_WIDTH, endPos, false);
            if (layout == null)
                break;
            y += layout.getAscent();
            layout.draw(graphics, 10, y);
        }
    }

    public void layout(List<String> lines) {
//        AttributedString string = new AttributedString("This is a test!");
//        string.addAttribute(TextAttribute.SUPERSCRIPT, TextAttribute.SUPERSCRIPT_SUPER, 5, 8);
//        graphics.drawString(string.getIterator(), 0.0f, 20.0f);

//        String text = "Your long text here: All good men must come to the aid of their party!";
//        AttributedString attributedString = new AttributedString(text);
//        attributedString.addAttribute(TextAttribute.FONT, new Font("Arial", Font.PLAIN, 14));
//        // Set font attributes if needed
//
//        // Create a TextLayout
//        FontRenderContext frc = new FontRenderContext(null, true, true);
//        TextLayout layout = new TextLayout(attributedString.getIterator(), frc);
//
//        // Set the desired wrapping width
//        float wrappingWidth = 20; // Adjust as needed
//
//        // Create a LineBreakMeasurer
//        AttributedCharacterIterator charIterator = attributedString.getIterator();
//        LineBreakMeasurer measurer = new LineBreakMeasurer(charIterator, frc);
//
//        // Iterate through the text and break lines
//        while (measurer.getPosition() < charIterator.getEndIndex()) {
//            layout = measurer.nextLayout(wrappingWidth);
//            graphics.drawString(layout.)
//            // Handle each line layout as needed
//            // ...
//        }

        String text = "This\nis a long text that needs to be wrapped into multiple\n\nlines. " +
                "We want to handle line breaks gracefully.";

        if (true) {
            render(text);
            return;
        }

        // Create a font (you can customize this)
        Font font = new Font("Arial", Font.PLAIN, 15);

        // Create an AttributedString from the text
        AttributedString richString = new AttributedString(text);
        // richString.addAttribute(TextAttribute.FONT, font);
        richString.addAttribute(TextAttribute.WEIGHT, TextAttribute.WEIGHT_BOLD); // , 11, text.length());
        richString.addAttribute(TextAttribute.WEIGHT, TextAttribute.WEIGHT_LIGHT, 20, text.length());
        richString.addAttribute(TextAttribute.SUPERSCRIPT, TextAttribute.SUPERSCRIPT_SUPER, 5, text.length());

        // Create a LineBreakMeasurer
        LineBreakMeasurer measurer = new LineBreakMeasurer(
                richString.getIterator(),
                new FontRenderContext(null, true, true));

        // Set the desired wrapping width
        float wrappingWidth = 150; // Adjust as needed

        // Draw each line
        int y = 0;
        while (true) {
            int endPos = measurer.nextOffset(wrappingWidth);
            if (endPos == -1)
                break;
            for (int i = measurer.getPosition(); i < endPos; i++) {
                if (text.charAt(i) == '\n') {
                    endPos = i + 1;
                    break;
                }
            }
            TextLayout layout = measurer.nextLayout(wrappingWidth, endPos, false);
            if (layout == null)
                break;
            y += layout.getAscent();
            layout.draw(graphics, 10, y);
        }
    }

    // Simple routine to substitute HTML entities into their displayable state:
    public static String transformEntities(String string) {
        while (true) {
            int tagStart = string.indexOf('&');
            if (tagStart == -1)
                return string;

            int tagEnd = string.indexOf(';', tagStart);
            if (tagEnd == -1)
                return string;

            String entity = string.substring(tagStart, tagEnd + 1).trim(); // Includes '&' and ';'
            String substitution = (ENTITY_MAP.get(entity) != null) ? ENTITY_MAP.get(entity) : "";
            string = string.substring(0, tagStart) + substitution + string.substring(tagEnd + 1);
        }
    }

    // Simple routine to strip all HTML-looking tags from a string:
    public static String stripTags(String string) {
        while (true) {
            int tagStart = string.indexOf('<');
            if (tagStart == -1)
                return string;

            int tagEnd = string.indexOf('>', tagStart);
            if (tagEnd == -1)
                return string;

            String tag = string.substring(tagStart + 1, tagEnd).trim();
            string = string.substring(0, tagStart) + string.substring(tagEnd + 1);
        }
    }
}

/**
 * This class implements a lightweight emulation of FTC Telemetry that can run on the PC.
 */
public class WilyTelemetry implements Telemetry {
    // Enable unit test:
    final boolean TEST = false;

    // Use this font for display on the PC. It's different from the sizing font because the
    // sizing font doesn't support the full unicode character set (like emojis):
    public final int DISPLAY_FONT_SIZE = 16;
    final Font PROPORTIONAL_DISPLAY_FONT = new Font(null, Font.PLAIN, DISPLAY_FONT_SIZE);
    final Font MONOSPACE_FONT = new Font(Font.MONOSPACED, Font.PLAIN, DISPLAY_FONT_SIZE);

    // Try to emulate the same line width as the REV Driver Station in its horizontal
    // configuration. Because the DS uses a proportional font, and because we don't have
    // access to the source code, we can't precisely replicate the DS behavior when a
    // single line is too wide for the DS and has to be broken into multiple lines. So
    // we use a different proportional font for measuring the text and just kind of guess.
    // We try to use a font that supplies similar proportions for different strings,
    // settling on the following and measuring its width using the strings
    // "123456789,123456789,123456789,123456789,1" and "WWWWWWWWW,WWWWWWWWW,WWWWWWW" which
    // are each as wide as can be displayed on the REV Control Hub without wrapping:
    final Font PROPORTIONAL_SIZING_FONT = new Font(null, Font.PLAIN, 10);
    final int LINE_WIDTH_IN_FONT_UNITS = 239;
    final int HEIGHT_IN_LINES = 18;

    // Line width when using the monospace font:
    final int LINE_WIDTH_IN_CHARACTERS = 50;

    // Global state:
    public static WilyTelemetry instance;

    // Class state:
    TelemetryWindow telemetryWindow;
    Canvas canvas;
    FontMetrics metrics;
    ArrayList<String> lineList = new ArrayList<>();
    DisplayFormat displayFormat = DisplayFormat.CLASSIC; // HTML vs. monospace modes

    // Unit test:
    @SuppressWarnings({"UnnecessaryUnicodeEscape", "StringConcatenationInLoop"})
    private void test(WilyTelemetry telemetry) {
        telemetry.addLine("This\uD83C\uDF85\uD83C\uDFFEhas\uD83D\uDD25emojis\uD83C\uDF1Ebetween\u2744\uFE0Fevery\uD83D\uDC14word");
        String emojis = ">";
        for (int i = 0; i < 30; i++) {
            emojis += ((i & 1) != 0) ? "\uD83C\uDF1E" : "\u2744\uFE0F"; // Surrogate vs. variation selector
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
        // Trim all trailing spaces:
        string = string.replaceAll("\\s+$", "");
        if (displayFormat == DisplayFormat.MONOSPACE) {
            return string.length();
        } else {
            return metrics.stringWidth(string);
        }
    }

    // Wily Works constructor for a Telemetry object:
    public WilyTelemetry() {
        instance = this;

        telemetryWindow = new TelemetryWindow("Telemetry", 400);
        telemetryWindow.setVisible(true);

        canvas = telemetryWindow.getCanvas();
        metrics = canvas.getBufferStrategy().getDrawGraphics().getFontMetrics(PROPORTIONAL_SIZING_FONT);
    }

    public Line addLine(String string) {
        if (lineList.size() <= HEIGHT_IN_LINES) {
            int newLineIndex;
            while ((newLineIndex = string.indexOf("\n")) != -1) {
                String line = string.substring(0, newLineIndex);
                lineList.add(line);
                string = string.substring(newLineIndex + 1);
            }
            lineList.add(string);
        }
        return null; // ###
    }

    @Override
    public boolean removeLine(Line line) {
        return false;
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setAutoClear(boolean autoClear) {

    }

    @Override
    public int getMsTransmissionInterval() {
        return 0;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {

    }

    @Override
    public String getItemSeparator() {
        return null;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {

    }

    @Override
    public String getCaptionValueSeparator() {
        return null;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {

    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        this.displayFormat = displayFormat;
    }

    @Override
    public Log log() {
        return null;
    }

    public Item addData(String caption, Object value) {
        addLine(String.format("%s : %s", caption, value.toString()));
        return null; // ###
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return null;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return null;
    }

    @Override
    public boolean removeItem(Item item) {
        return false;
    }

    public Item addData(String caption, String format, Object... args) {
        addData(caption, String.format(format, args));
        return null; // ###
    }

    public Line addLine() { return addLine(""); }
    public void clear() { lineList.clear(); }
    public void clearAll() { lineList.clear(); }

    @Override
    public Object addAction(Runnable action) {
        return null;
    }

    @Override
    public boolean removeAction(Object token) {
        return false;
    }

    @Override
    public void speak(String text) {

    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {

    }

    private void simpleFlow(Graphics g) {
        int lineWidth;
        if (displayFormat == DisplayFormat.MONOSPACE) {
            g.setFont(MONOSPACE_FONT);
            lineWidth = LINE_WIDTH_IN_CHARACTERS;
        } else {
            g.setFont(PROPORTIONAL_DISPLAY_FONT);
            lineWidth = LINE_WIDTH_IN_FONT_UNITS;
        }

        int y = HEIGHT_IN_LINES;
        int lineCount = 0;
        for (String line : lineList) {
            if (displayFormat == DisplayFormat.HTML) {
                line = Layout.stripTags(line);
                line = Layout.transformEntities(line);
            }

            while (lineCount < HEIGHT_IN_LINES) {
                int lineBreak = line.length();
                if (stringWidth(line) > lineWidth) {
                    // If the line is too long, try and break at a space:
                    lineBreak = line.length() - 1;
                    while ((lineBreak > 0) &&
                            ((line.charAt(lineBreak - 1) != ' ') || // -1 to avoid space on next line
                                    (stringWidth(line.substring(0, lineBreak - 1)) > lineWidth)))
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
                                    (stringWidth(line.substring(0, lineBreak)) <= lineWidth))
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
    }

    public boolean update() {
        Graphics2D g = (Graphics2D) canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());

        if (TEST) {
            System.out.println(stringWidth("123456789,123456789,123456789,123456789,12"));
            System.out.println(stringWidth("WWWWWWWWW,WWWWWWWWW,WWWWWWWW"));
            test(this);
        }

        if (displayFormat == DisplayFormat.HTML) {
            Layout html = new Layout(g, displayFormat);
            html.layout(null);
        } else {
            simpleFlow(g);
        }

        g.dispose();
        canvas.getBufferStrategy().show();
        lineList.clear();

        return true; // The transmission occurred successfully
    }
}
