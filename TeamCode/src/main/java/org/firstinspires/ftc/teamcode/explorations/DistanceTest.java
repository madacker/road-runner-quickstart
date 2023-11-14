package org.firstinspires.ftc.teamcode.explorations;

import android.graphics.PointF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PoseMessage;

import java.util.LinkedList;

@TeleOp(name="DistanceTest",group="Explore")
public class DistanceTest extends LinearOpMode {

    private final LinkedList<PointF> distanceHistory = new LinkedList<>();

    public void drawDistances(Canvas c, Pose2d pose, double distance) {
        final double DISTANCE_SENSOR_OFFSET = 8; // Offset from sensor to center of robot

        if (distance >= 0) {
            distance += DISTANCE_SENSOR_OFFSET;

            double theta = pose.heading.log();
            double x = pose.position.x + distance * Math.cos(theta);
            double y = pose.position.y + distance * Math.sin(theta);

            distanceHistory.add(new PointF((float) x, (float) y));
            if (distanceHistory.size() > 100) {
                distanceHistory.removeFirst();
            }
        }

        c.setStrokeWidth(1);
        c.setFill("#808080");
        for (PointF point: distanceHistory) {
            c.fillRect(point.x - 1, point.y - 1, 3, 3);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        waitForStart();

        while (opModeIsActive()) {
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);

            drive.setDrivePowers(powers);
            drive.updatePoseEstimate();

            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            if (distance == distanceSensor.distanceOutOfRange)
                distance = -1;

            telemetry.addData("distance", distance);
            telemetry.addData("distanceOutOfRange", distanceSensor.distanceOutOfRange);
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
    }
}
