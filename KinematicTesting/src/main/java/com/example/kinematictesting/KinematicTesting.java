package com.example.kinematictesting;

import com.example.kinematictesting.framework.Simulation;

class MecanumDrive {

}

public class KinematicTesting {
    public static void main(String[] args)
    {
        Simulation simulation = new Simulation();
        MecanumDrive mecanumDrive = new MecanumDrive();
        Drive drive = new Drive(mecanumDrive);

        while (true) {
            drive.update();
            simulation.update();
        }
    }
}

class Drive {
    MecanumDrive drive;
    Drive(MecanumDrive drive) {
        this.drive = drive;
    }
    void update() {
//        PoseVelocity2d powers = new PoseVelocity2d(
//                new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
//                -gamepad1.right_stick_x);
//
//        drive.updatePoseEstimate();
//        drive.setDrivePowers(powers);
//
//        telemetry.addData("x", drive.pose.position.x);
//        telemetry.addData("y", drive.pose.position.y);
//        telemetry.addData("heading", drive.pose.heading);
//        telemetry.update();
//
//        // Code added to draw the pose:
//        TelemetryPacket p = new TelemetryPacket();
//        Canvas c = p.fieldOverlay();
//        c.setStroke("#3F51B5");
//        MecanumDrive.drawRobot(c, drive.pose);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboard.sendTelemetryPacket(p);
    }
}