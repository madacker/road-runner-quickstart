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

@TeleOp
public class Driver extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TimeSplitter startupTime = TimeSplitter.create("Startup time");
        TimeSplitter loopTime = TimeSplitter.create("Loop time");

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

            // Draw the pose:
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // Update the telemetry pose and update the LED loop:
            drive.updatePoseEstimate();
            led.update();

            // Refine the pose:
            Pose2d refinedPose = refiner.refinePose(drive.pose, canvas);
            if (refinedPose != null) {
                drive.pose = refinedPose;
                led.setSteadyColor(Led.Color.GREEN);
                led.setPulseColor(Led.Color.RED, 0.25);
            }

            // Draw the pose:
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

