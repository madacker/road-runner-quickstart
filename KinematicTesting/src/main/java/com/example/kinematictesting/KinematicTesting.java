package com.example.kinematictesting;

import com.acmerobotics.roadrunner.Pose2d;
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
    public static void main(String[] args)
    {
        Simulation simulation = new Simulation();
        Gamepad gamepad = new Gamepad();
        Telemetry telemetry = new Telemetry();
        MecanumDrive mecanumDrive = new MecanumDrive(new Pose2d(0, 0, 0));
        Drive drive = new Drive(mecanumDrive, gamepad, telemetry);

        while (true) {
            simulation.update();
            gamepad.update();
            drive.update();
        }
    }
}

class Drive {
    MecanumDrive drive;
    Gamepad gamepad1;
    Telemetry telemetry;
    Drive(MecanumDrive drive, Gamepad gamepad1, Telemetry telemetry) {
        this.drive = drive;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }
    void update() {
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