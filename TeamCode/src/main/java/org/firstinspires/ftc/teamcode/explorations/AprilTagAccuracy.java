package org.firstinspires.ftc.teamcode.explorations;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="AprilTagAccuracy",group="Explore")
public class AprilTagAccuracy extends LinearOpMode {

    @Override
    public void runOpMode() {
        Globals.initialize(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            Globals.startLoop();
            drive.updatePoseEstimate();
            boolean doingSweep = drive.doActionsWork(drive.pose, drive.poseVelocity, Globals.packet);
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
                Globals.canvas.setStroke("#3F51B5");
                MecanumDrive.drawRobot(Globals.canvas, drive.pose);
            }

            Globals.endLoop();
        }

        TimeSplitter.reportAllResults();
    }
}
