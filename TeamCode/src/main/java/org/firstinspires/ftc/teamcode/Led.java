package org.firstinspires.ftc.teamcode;

import static java.lang.System.nanoTime;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Led {
    public enum Color { OFF, RED, GREEN };

    private Color pulseColor = Color.OFF;
    private Color steadyColor = Color.OFF;
    private double pulseEndTime = -1.0; // Seconds

    private DigitalChannel redLed;
    private DigitalChannel greenLed;

    public Led(HardwareMap hardwareMap) {
        redLed = hardwareMap.get(DigitalChannel.class, "red");
        greenLed = hardwareMap.get(DigitalChannel.class, "green");

        // The mode defaults to 'input':
        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);

        setSteadyColor(Color.OFF);
    }

    private void setColor(Color color) {
        boolean redOn = false;
        boolean greenOn = false;
        if (color == Color.RED) {
            redOn = true;
        } else if (color == Color.GREEN) {
            greenOn = true;
        }
        redLed.setState(!redOn);
        greenLed.setState(!greenOn);
    }

    private boolean isPulseActive() {
        return ((pulseEndTime != -1.0) && (nanoTime() * 1e-9 < pulseEndTime));
    }

    public void setSteadyColor(Color color) {
        steadyColor = color;
        if (!isPulseActive())
            setColor(color);
    }

    public void setPulseColor(Color color, double  seconds) {
        pulseColor = color;
        pulseEndTime = nanoTime() * 1e-9 + seconds; // Seconds
        setColor(color);
    }

    public void update() {
        if ((pulseEndTime != -1.0) && (!isPulseActive())) {
            pulseEndTime = -1.0;
            setColor(steadyColor);
        }
    }
}
