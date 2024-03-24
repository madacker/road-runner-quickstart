package org.firstinspires.ftc.teamcode.explorations;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.LinkedList;

@TeleOp(name="DistanceTest",group="Explore")
public class DistanceTest extends LinearOpMode {

    private final LinkedList<Point> distanceHistory = new LinkedList<>();

    public void drawDistances(Canvas c, Pose2d pose, double distance) {
        final double DISTANCE_SENSOR_OFFSET = 1; // Offset from sensor to center of robot

        if (distance >= 0) {
            distance += DISTANCE_SENSOR_OFFSET;

            double theta = pose.heading.log() + Math.PI; // +180 degrees because facing backwards
            double x = pose.position.x + distance * Math.cos(theta);
            double y = pose.position.y + distance * Math.sin(theta);

            distanceHistory.add(new Point(x, y));
            if (distanceHistory.size() > 100) {
                distanceHistory.removeFirst();
            }
        }

        c.setStrokeWidth(1);
        c.setFill("#808080");
        for (Point point: distanceHistory) {
            c.fillRect(point.x - 1, point.y - 1, 3, 3);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Globals globals = new Globals(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), globals);
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        TimeSplitter timer = TimeSplitter.create("getDistance");

        waitForStart();

        while (opModeIsActive()) {
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);

            drive.setDrivePowers(powers);
            drive.updatePoseEstimate();

            timer.startSplit();
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            timer.endSplit();
            if (distance == distanceSensor.distanceOutOfRange)
                distance = -1;

            telemetry.addData("distance", distance);
            // telemetry.addData("distanceOutOfRange", distanceSensor.distanceOutOfRange);
            telemetry.update();

            // Begin drawing:
            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();

            // Draw the distance results:
            drawDistances(c, drive.pose, distance);

            // Draw the robot pose:
            c.setStroke("#3F51B5");
            MecanumDrive.drawRobot(c, drive.pose);

            // Done drawing:
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(p);
        }

        TimeSplitter.reportAllResults();
    }
}
