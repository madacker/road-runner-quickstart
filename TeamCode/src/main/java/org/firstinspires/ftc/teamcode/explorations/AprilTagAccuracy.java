package org.firstinspires.ftc.teamcode.explorations;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Led;
import org.firstinspires.ftc.teamcode.Refiner;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="AprilTagAccuracy",group="Explore")
public class AprilTagAccuracy extends LinearOpMode {

    @Override
    public void runOpMode() {
        Refiner refiner = new Refiner(hardwareMap, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            // Set up for visualizations:
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            boolean doingSweep = drive.doActionsWork(packet);
            if (!doingSweep) {
                if (gamepad1.a) {
                    double SWEEP_ANGLE = 80; // Degrees
                    Action sweep = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.log()))
                            .turn(Math.toRadians(-SWEEP_ANGLE/2))
                            .turn(Math.toRadians(SWEEP_ANGLE), new TurnConstraints(0.3, -3.14, 3.14))
                            .turn(Math.toRadians(-SWEEP_ANGLE/2))
                            .build();
                    drive.runParallel(sweep);
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));
                }
            }

            // Draw the best estimate pose:
            if (!doingSweep) {
                canvas.setStroke("#3F51B5");
                MecanumDrive.drawRobot(canvas, drive.pose);
            }

            // Draw the pose history:
            drive.updatePoseEstimate();
            Pose2d refinedPose = refiner.refinePose(drive.pose, drive, canvas);
            if (refinedPose != null) {
                drive.recordPose(refinedPose, 0);

                // Don't reset the pose while doing a sweep:
                if (!doingSweep) {
                    drive.pose = refinedPose;
                }
            }
            drive.drawPoseHistory(canvas);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
