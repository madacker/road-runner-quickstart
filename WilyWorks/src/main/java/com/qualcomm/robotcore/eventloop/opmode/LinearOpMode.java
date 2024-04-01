package com.qualcomm.robotcore.eventloop.opmode;

public abstract class LinearOpMode extends OpMode {

    abstract public void runOpMode() throws InterruptedException;

    public void waitForStart() {
    }

    public final boolean opModeIsActive() {
        return true;
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public final boolean isStopRequested() {
        return false;
    }
}
