package com.example.kinematictesting.framework;

public class Telemetry {
    public boolean update() { return true; }
    public void addData(String caption, String format, Object... args) {}
    public void addData(String caption, Object value) {}
    public void addLine() {}
    public void addLine(String lineCaption) {}
}
