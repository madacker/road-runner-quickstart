package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="Driver", group="Aardvark")
public class Driver extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TimeSplitter startupTime = TimeSplitter.create("Startup time");
        TimeSplitter loopTime = TimeSplitter.create("Loop time");
        boolean initializedPose = false;

        startupTime.startSplit();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Refiner refiner = new Refiner(hardwareMap);
        Led led = new Led(hardwareMap);
        startupTime.endSplit();

        waitForStart();

        while (opModeIsActive()) {
            loopTime.startSplit();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // Update the telemetry pose and update the LED loop:
            drive.updatePoseEstimate();
            led.update();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // Draw the uncorrected reference pose:
            canvas.setStroke("#a0a0a0");
            MecanumDrive.drawRobot(canvas, drive.pose2);

            // Draw AprilTag poses and refine them:
            Pose2d refinedPose = refiner.refinePose(drive.pose, canvas);
            if (refinedPose != null) {
                led.setSteadyColor(Led.Color.GREEN);
                led.setPulseColor(Led.Color.RED, 0.25);
                drive.pose = refinedPose;

                // Pose2 is used to evaluate drift over the course of a driving session.
                // It starts the same as the initial pose set but is never again updated
                // by the pose refinement:
                if (!initializedPose) {
                    drive.pose2 = refinedPose;
                    initializedPose = true;
                }
            }

            // Draw the best estimate pose:
            canvas.setStroke("#3F51B5");
            MecanumDrive.drawRobot(canvas, drive.pose);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            loopTime.endSplit();
        }

        // Cleanup:
        refiner.close();
        TimeSplitter.logAllResults();
        led.setSteadyColor(Led.Color.OFF);
    }
}

