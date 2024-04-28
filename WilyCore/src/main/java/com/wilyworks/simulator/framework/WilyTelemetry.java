package com.wilyworks.simulator.framework;

import static java.awt.Font.MONOSPACED;
import static java.awt.font.TextAttribute.POSTURE_REGULAR;
import static java.awt.font.TextAttribute.WEIGHT_REGULAR;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.Telemetry.DisplayFormat;

import android.annotation.SuppressLint;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Paint;
import java.awt.font.FontRenderContext;
import java.awt.font.LineBreakMeasurer;
import java.awt.font.TextAttribute;
import java.awt.font.TextLayout;
import java.math.BigInteger;
import java.text.AttributedCharacterIterator;
import java.text.AttributedString;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Helper class for handling simplified HTML display.
 */
class Layout {
    private static final int LINE_WIDTH = 239; // Line width, in pixel units
    private static final int FONT_SIZE = 16; // Default font size
    private static final float[] HEADING_MULTIPLES = { // Heading size multiples
            1.5f, 1.4f, 1.3f, 1.2f, 1.1f, 1f,
    };
    private Graphics2D graphics; // Graphics context
    private DisplayFormat displayFormat; // CLASSIC, MONOSPACE or HTML

    // Track tag attributes as we accumulate the string buffer:
    static class Attribute {
        AttributedCharacterIterator.Attribute attribute; // addAttribute() attribute
        Object value; // addAttribute() value
        int pos; // Position in the buffer

        public Attribute(AttributedCharacterIterator.Attribute attribute, Object value, int pos) {
            this.attribute = attribute; this.value = value; this.pos = pos;
        }
    }

    // Track line breaks as we accumulate the string buffer:
    static class LineBreak {
        int pos; // Position in the buffer
        int verticalOffset; // Vertical offset, zero is the default

        public LineBreak(int pos, int verticalOffset) {
            this.pos = pos; this.verticalOffset = verticalOffset;
        }
    }

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

    Layout(Graphics2D graphics, DisplayFormat displayFormat) {
        this.graphics = graphics;
        this.displayFormat = displayFormat;
    }


//    // Transform @@@
//    private static AttributedString getAttributedString(String text, ArrayList<Tag> tags) {
//        AttributedString string = new AttributedString(text);
//        for (Tag tag: tags) {
//            string.addAttribute(tag.attribute, tag.value, tag.pos, text.length());
//        }
//        return string;
//    }

//    // Transform all HTML 'entities' (like &nbsp; into a space character):
//    private static String substituteEntities(String text) {
//        StringBuilder builder = new StringBuilder();
//        int loopStart = 0; // Index into input text to start the next loop iteration
//        int addedCount = 0; // Running total of characters that have been added from substitutions
//        int tagIndex = 0; // Current tag index for fixups
//
//        while (loopStart < text.length()) {
//            int entityStart = text.indexOf('&', loopStart);
//            int entityEnd = text.indexOf(';', entityStart) + 1; // Make it exclusive
//            if ((entityStart == -1) || (entityEnd == 0)) {
//                // A valid entity wasn't found so set conditions to terminate the loop:
//                entityStart = text.length();
//                entityEnd = text.length();
//            }
//            builder.append(text.substring(loopStart, entityStart));
//            loopStart = entityEnd; // Prepare for next loop iteration
//
//            // Now do the actual entity substitution:
//            String entity = text.substring(entityStart, entityEnd).trim(); // Includes '&' and ';'
//            addedCount -= entity.length(); // Remove all of the entity characters
//            String substitution = ENTITY_MAP.get(entity);
//            if (substitution != null) {
//                addedCount += substitution.length(); // Account for new characters we're adding
//                builder.append(substitution);
//            }
//        }
//        return builder.toString();
//    }

    // Returns true if the current line is empty:
    boolean isEmptyLine(StringBuilder builder, List<LineBreak> lineBreaks) {
        if ((builder.length() == 0) || (lineBreaks.size() == 0))
            return true;

        LineBreak lastBreak = lineBreaks.get(lineBreaks.size() - 1);
        return lastBreak.pos == builder.length();
    }

//    ArrayList<Tag> getTags(String text) {
//        StringBuilder builder = new StringBuilder();
//        ArrayList<Tag> tags = null;
//
//        int loopStart = 0;
//        while (loopStart < text.length()) {
//            int tagStart = text.indexOf('<', loopStart);
//            int tagEnd = text.indexOf('>', tagStart) + 1; // Make it exclusive
//            if ((tagStart == -1) || (tagEnd == 0)) {
//                tagStart = text.length();
//                tagEnd = text.length();
//            }
//            builder.append(text.substring(loopStart, tagStart));
//            loopStart = tagEnd; // Prepare for next loop iteration
//
//            String tag = text.substring(tagStart, tagEnd).trim(); // Includes '<' and '>'
//            String element = tag;
//            int space = text.indexOf(' ');
//            if (space != -1)
//                element = tag.substring(0, space);
//
//            // https://docs.oracle.com/javase/8/docs/api/java/awt/font/TextAttribute.html#SIZE
//            switch (tag) {
//                case "<br>": builder.append("\n"); break;
//                case "<tt>": tags.add(new Tag(TextAttribute.FAMILY, "MONOSPACE", builder.length())); break;
//                case "</tt>": tags.add(new Tag(TextAttribute.FAMILY, "SANS_SERIF", builder.length())); break; // @@@ Initialize?
//                case "<div>":
//                    if (!isEmptyLine(builder))
//                        builder.append("\n");
//                    break;
//                case "</div>": builder.append("\n"); break;
//                case "<font":
//
//
//
//            }
//        }
//    }

//    public void render(String text) {
//        // Want to parse before converting entities
//        // Entity removal needs to remap everything
//
//        String plainText = text;
//        if (displayFormat == HTML) {
//            plainText = stripTags(transformEntities(text));
//        }
//
//        AttributedString string = new AttributedString(plainText);
//        string.addAttribute(TextAttribute.SIZE, WilyTelemetry.instance.DISPLAY_FONT_SIZE);
//        if (displayFormat == Telemetry.DisplayFormat.MONOSPACE) {
//            string.addAttribute(TextAttribute.FAMILY, "MONOSPACED");
//        }
//        if (displayFormat == HTML) {
//            // addHtmlAttributes(text, string);
//        }
//
//        LineBreakMeasurer measurer = new LineBreakMeasurer(
//                string.getIterator(),
//                new FontRenderContext(null, true, true));
//
//        // Draw each line
//        int y = 0;
//        while (true) {
//            int endPos = measurer.nextOffset(LINE_WIDTH);
//            if (endPos == -1)
//                break;
//            for (int i = measurer.getPosition(); i < endPos; i++) {
//                if (text.charAt(i) == '\n') {
//                    endPos = i + 1;
//                    break;
//                }
//            }
//            TextLayout layout = measurer.nextLayout(LINE_WIDTH, endPos, false);
//            if (layout == null)
//                break;
//            y += layout.getAscent();
//            layout.draw(graphics, 10, y);
//        }
//    }

    // Render what we've accumulated:
    private void renderText() {
        // Convert our accumulation buffer to an AttributeString:
        String text = buffer.toString();
        AttributedString attributedString = new AttributedString(text);
        for (Attribute attribute : attributes) {
            attributedString.addAttribute(attribute.attribute, attribute.value, attribute.pos, text.length());
        }

        // Render with word wrapping:
        LineBreakMeasurer measurer = new LineBreakMeasurer(
                attributedString.getIterator(),
                new FontRenderContext(null, true, true));

        Iterator<LineBreak> iterator = lineBreaks.iterator();
        LineBreak lineBreak = iterator.next();
        float yCurrent = -1.0f;

        while (true) {
            // Find where the measurer wants to put the next line break and compare that to
            // where we want to put the next line break:
            int layoutEndPos = measurer.nextOffset(LINE_WIDTH);
            int newLines = 0;
            int verticalOffset = 0;

            // If the layout line-end is less than the next explicitly requested newline, then
            // add one implicit newline:
            if (lineBreak.pos >= layoutEndPos) {
                newLines = 1;

                // If the explicitly requested newline happens to be the same as the natural
                // layout line-end, we can ignore the explicit request:
                if (layoutEndPos == lineBreak.pos) {
                    lineBreak = iterator.next();
                }
            }

            // If the explicitly requested newline comes before the natural layout line-end,
            // then make that the new line. Use a loop to handle multiple explicit newlines
            // in a row:
            while (lineBreak.pos <= layoutEndPos) {
                layoutEndPos = lineBreak.pos;
                verticalOffset += lineBreak.verticalOffset;
                newLines++;
                lineBreak = iterator.next();
            }

            TextLayout layout = measurer.nextLayout(LINE_WIDTH, layoutEndPos, false);
            if (layout == null)
                break;

            // Handle the offset for the very first line:
            if (yCurrent == -1.0f)
                yCurrent = layout.getAscent();

            // Draw the text:
            layout.draw(graphics, 0, yCurrent);

            // Advance 'y' for the next line:
            float leading = layout.getAscent() + layout.getDescent();
            yCurrent += newLines * leading + verticalOffset;
        }
    }

    DisplayFormat format;
    StringBuilder buffer; //
    ArrayList<Attribute> attributes;
    ArrayList<LineBreak> lineBreaks;

    // History structure for tags:
    static public class ColorRecord {
        Paint foreground;
        Paint background;

        public ColorRecord(Paint foregroundColor, Paint backgroundColor) {
            this.foreground = foregroundColor; this.background = backgroundColor;
        }
    }

    public void parseAndRender(Graphics2D graphics, DisplayFormat format, String text) {
        this.graphics = graphics;
        this.format = format;
        this.buffer = new StringBuilder();
        this.attributes = new ArrayList<>();
        this.lineBreaks = new ArrayList<>();

text = "This is a <b>long</b> text that needs to be wrapped into multiple lines.   \n\n" +
        "We want to handle line breaks gracefully.";

        // \n
        Pattern newlineSearchPattern = Pattern.compile("(\\n)");

        // <big>, &nbsp;, \n
        // For a tag, group2 = element name, group3 = arguments (needs trimming)
        Pattern htmlSearchPattern = Pattern.compile("(\\n|&.*?;|<\\s*?([/\\w]+)(.*?)>)");

        // style='color: 0xffffff; background: 0x3e3e3e;'
        Pattern spanColorPattern = Pattern.compile(
            "\\s*?style\\s*?=\\s*?['|\"].*?color\\s*?:\\s*?(?:0x|#)([0-9a-fA-F]+)");
        Pattern spanBackgroundPattern = Pattern.compile(
            "\\s*?style\\s*?=\\s*?['|\"].*?background\\s*?:\\s*?(?:0x|#)([0-9a-fA-F]+)");

        // <font color='#00ff00'>
        Pattern fontColorPattern = Pattern.compile(
            "\\s*scolor\\s*?=\\s*?['|\"](?:0x|#)([0-9a-fA-F]+)>");

        Pattern searchPattern = (displayFormat == DisplayFormat.HTML) ? htmlSearchPattern : newlineSearchPattern;

        LinkedList<String> familyStack = new LinkedList<>();
        LinkedList<Float> weightStack = new LinkedList<>();
        LinkedList<Float> postureStack = new LinkedList<>();
        LinkedList<Float> sizeStack = new LinkedList<>();
        LinkedList<Integer> superscriptStack = new LinkedList<>();
        LinkedList<ColorRecord> colorStack = new LinkedList<>();
        LinkedList<Integer> underlineStack = new LinkedList<>();
        LinkedList<Boolean> strikeThroughStack = new LinkedList<>();

        String family = (displayFormat == DisplayFormat.MONOSPACE) ? MONOSPACED : "Default";
        float weight = WEIGHT_REGULAR;
        float posture = POSTURE_REGULAR;
        float size = FONT_SIZE;
        int superscript = 0;
        Paint foreground = new Color(0xffffff); // The standard text color
        Paint background = new Color(0x390708); // The reddish-brown of the driver station
        int underline = -1;
        boolean strikeThrough = false;

        attributes.add(new Attribute(TextAttribute.FAMILY, family, 0));
        attributes.add(new Attribute(TextAttribute.WEIGHT, weight, 0));
        attributes.add(new Attribute(TextAttribute.POSTURE, posture, 0));
        attributes.add(new Attribute(TextAttribute.SIZE, size, 0));
        attributes.add(new Attribute(TextAttribute.SUPERSCRIPT, superscript, 0));
        attributes.add(new Attribute(TextAttribute.FOREGROUND, foreground, 0));
        attributes.add(new Attribute(TextAttribute.BACKGROUND, background, 0));
        attributes.add(new Attribute(TextAttribute.UNDERLINE, underline, 0));
        attributes.add(new Attribute(TextAttribute.STRIKETHROUGH, strikeThrough, 0));

        // Clear the background using the default background color:
        graphics.setPaint(background);
        graphics.fillRect(0, 0, 10000, 10000);

        // Start parsing!
        Matcher matcher = searchPattern.matcher(text);
        int previousSearchEnd = 0;
        while (matcher.find()) {
            buffer.append(text, previousSearchEnd, matcher.start());
            previousSearchEnd = matcher.end();
            switch (matcher.group().charAt(0)) {
                case '\n':
                    lineBreaks.add(new LineBreak(buffer.length(), 0));
                    break;

                case '&':
                    String entitySubstitution = ENTITY_MAP.get(matcher.group());
                    if (entitySubstitution != null) {
                        buffer.append(entitySubstitution);
                    }
                    break;

                case '<':
                    String element = matcher.group(2);
                    String tagArguments = matcher.group(3);
                    switch (element) {
                        // Line tags:
                        case "br":
                            lineBreaks.add(new LineBreak(buffer.length(), 0));
                            break;
                        case "div":
                            if (!isEmptyLine(buffer, lineBreaks))
                                lineBreaks.add(new LineBreak(buffer.length(), 0));
                            break;
                        case "/div":
                            lineBreaks.add(new LineBreak(buffer.length(), 0));
                            break;
                        case "h1": case "h2": case "h3": case "h4": case "h5": case "h6":
                            sizeStack.push(size);
                            size = HEADING_MULTIPLES[element.charAt(1) - '1'] * FONT_SIZE;
                            attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                            lineBreaks.add(new LineBreak(buffer.length(), 0));
                            break;
                        case "/h1": case "/h2": case "/h3": case "/h4": case "/h5": case "/h6":
                            if (sizeStack.size() != 0) {
                                size = sizeStack.pop();
                                attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                                lineBreaks.add(new LineBreak(buffer.length(), 0));
                            }
                            break;

                        // Text attributes:
                        case "span":
                            colorStack.push(new ColorRecord(foreground, background));
                            Matcher spanColorMatch = spanColorPattern.matcher(tagArguments);
                            if (spanColorMatch.find()) {
                                BigInteger rgb = new BigInteger(spanColorMatch.group(), 16);
                                foreground = new Color(rgb.intValue());
                                attributes.add(new Attribute(TextAttribute.FOREGROUND, foreground, buffer.length()));
                            }
                            Matcher spanBackgroundMatch = spanColorPattern.matcher(tagArguments);
                            if (spanBackgroundMatch.find()) {
                                BigInteger rgb = new BigInteger(spanBackgroundMatch.group(), 16);
                                background = new Color(rgb.intValue());
                                attributes.add(new Attribute(TextAttribute.BACKGROUND, background, buffer.length()));
                            }
                            break;
                        case "font":
                            colorStack.push(new ColorRecord(foreground, background));
                            Matcher fontColorMatch = fontColorPattern.matcher(tagArguments);
                            if (fontColorMatch.find()) {
                                BigInteger rgb = new BigInteger(fontColorMatch.group(), 16);
                                foreground = new Color(rgb.intValue());
                                attributes.add(new Attribute(TextAttribute.FOREGROUND, foreground, buffer.length()));
                            }
                            break;
                        case "/span":
                        case "/font":
                            if (colorStack.size() != 0) {
                                ColorRecord node = colorStack.pop();
                                foreground = node.foreground;
                                background = node.background;
                                attributes.add(new Attribute(TextAttribute.FOREGROUND, foreground, buffer.length()));
                                attributes.add(new Attribute(TextAttribute.BACKGROUND, background, buffer.length()));
                            }
                            break;

                        case "big":
                            sizeStack.push(size);
                            size *= 1.25;
                            attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                            break;
                        case "small":
                            sizeStack.push(size);
                            size *= 0.8;
                            attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                            break;
                        case "/small":
                        case "/big":
                            if (sizeStack.size() != 0) {
                                size = sizeStack.pop();
                                attributes.add(new Attribute(TextAttribute.SIZE, size, buffer.length()));
                            }
                            break;

                        case "tt":
                            familyStack.push(family);
                            family = MONOSPACED;
                            attributes.add(new Attribute(TextAttribute.FAMILY, family, buffer.length()));
                            break;
                        case "/tt":
                            if (familyStack.size() != 0) {
                                family = familyStack.pop();
                                attributes.add(new Attribute(TextAttribute.FAMILY, familyStack.pop(), buffer.length()));
                            }
                            break;

                        case "b":
                            weightStack.push(weight);
                            weight = TextAttribute.WEIGHT_BOLD;
                            attributes.add(new Attribute(TextAttribute.WEIGHT, weight, buffer.length()));
                            break;
                        case "/b":
                            if (weightStack.size() != 0) {
                                weight = weightStack.pop();
                                attributes.add(new Attribute(TextAttribute.WEIGHT, weight, buffer.length()));
                            }
                            break;

                        // <i> - italics
                        // <b> - bold
                        // <u> - underline
                        // <strike> - strikethrough
                        // <sup>
                        // <sub>
                    }
            }
        }

        buffer.append(text, previousSearchEnd, text.length());
        lineBreaks.add(new LineBreak(text.length(), 0));
        lineBreaks.add(new LineBreak(Integer.MAX_VALUE, 0)); // Terminator
        renderText();
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

        StringBuilder textBuilder = new StringBuilder();
        for (String line: lines) {
            textBuilder.append(line);
        }
        String text = textBuilder.toString();

        if (true) {
            parseAndRender(graphics, displayFormat, text);
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
    final Font MONOSPACE_FONT = new Font(MONOSPACED, Font.PLAIN, DISPLAY_FONT_SIZE);

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
    @SuppressLint("DefaultLocale")
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
            html.layout(lineList);
        } else {
            simpleFlow(g);
        }

        g.dispose();
        canvas.getBufferStrategy().show();
        lineList.clear();

        return true; // The transmission occurred successfully
    }
}
