package com.example.kinematictesting;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.kinematictesting.framework.Canvas;
import com.example.kinematictesting.framework.FtcDashboard;
import com.example.kinematictesting.framework.Gamepad;
import com.example.kinematictesting.framework.MecanumDrive;
import com.example.kinematictesting.framework.Simulation;
import com.example.kinematictesting.framework.Telemetry;
import com.example.kinematictesting.framework.TelemetryPacket;

public class KinematicTesting {
    static final double DELTA_T = 0.100; // 100ms

    static double time() {
        return System.currentTimeMillis() / 1000.0;
    }

    public static void main(String[] args)
    {
        Simulation simulation = new Simulation();
        MecanumDrive mecanumDrive = new MecanumDrive(simulation);
        Gamepad gamepad = new Gamepad();
        Telemetry telemetry = new Telemetry();
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
    MecanumDrive drive;
    Gamepad gamepad1;
    Telemetry telemetry;

    Navigation(MecanumDrive drive, Gamepad gamepad1, Telemetry telemetry) {
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
        telemetry.update();

        // Code added to draw the pose:
        TelemetryPacket p = new TelemetryPacket();
        Canvas c = p.fieldOverlay();
        c.setStroke("#3F51B5");
        MecanumDrive.drawRobot(c, drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}