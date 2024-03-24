package com.example.wilyworks;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.example.wilyworks.framework.WilyMecanumDrive;
import com.example.wilyworks.framework.Simulation;
import com.example.wilyworks.framework.WilyTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

public class WilyWorks {
    static final double DELTA_T = 0.100; // 100ms

    static double time() {
        return System.currentTimeMillis() / 1000.0;
    }

    public static void main(String[] args)
    {
        Simulation simulation = new Simulation();
        WilyMecanumDrive mecanumDrive = new WilyMecanumDrive(simulation);
        Gamepad gamepad = new Gamepad();
        WilyTelemetry telemetry = new WilyTelemetry();
        Navigation loop = new Navigation(mecanumDrive, gamepad, telemetry);

        while (true) {
            // We use a fixed time quanta to allow single-stepping in the debugger:
            double time = time();
            gamepad.update();
            loop.update(DELTA_T);
            simulation.update(DELTA_T);

            // Busy-loop to consume any remaining time:
            while (time() - time < DELTA_T)
                ;
        }
    }
}

// Replace this class with your own.
class Navigation {
    WilyMecanumDrive drive;
    Gamepad gamepad1;
    WilyTelemetry telemetry;

    Navigation(WilyMecanumDrive drive, Gamepad gamepad1, WilyTelemetry telemetry) {
        this.drive = drive;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }
    void update(double deltaT) { // Time in seconds
        PoseVelocity2d powers = new PoseVelocity2d(
                new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                -gamepad1.right_stick_x);

        drive.updatePoseEstimate();
        drive.setDrivePowers(powers);

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.update();

        // Code added to draw the pose:
        TelemetryPacket p = new TelemetryPacket();
        Canvas c = p.fieldOverlay();
        c.setStroke("#3F51B5");
        WilyMecanumDrive.drawRobot(c, drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}