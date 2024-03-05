package org.firstinspires.ftc.teamcode.opticaltracking;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="OpticalTrackingTest",group="Explore")
public class OpticalTrackingTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.initialize(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        OpticalTrackingPaa5100 optical = hardwareMap.get(OpticalTrackingPaa5100.class, "optical2");
        Pose2d opticalPose = new Pose2d(0, 0, 0);
        TimeSplitter opticalTimer = TimeSplitter.create("Optical read");

        waitForStart();

        while (opModeIsActive()) {
            Globals.startLoop();

            // Move the robot according to the gamepad input:
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);
            drive.setDrivePowers(powers);

            // Query the optical motion:
            opticalTimer.startSplit();
            OpticalTrackingPaa5100.Motion motion = optical.getMotion();
            opticalTimer.endSplit();


            // Update and draw the optical motion pose estimate:
            double yaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            Vector2d fieldTranslation = new Vector2d(
                Math.cos(yaw) * motion.x - Math.sin(yaw) * motion.y,
                Math.sin(yaw) * motion.x + Math.cos(yaw) * motion.y);
            opticalPose = new Pose2d(opticalPose.position.plus(fieldTranslation), yaw);
            Globals.canvas.setStroke("#00ff00");
            MecanumDrive.drawRobot(Globals.canvas, opticalPose);

            // Output the results:
            Globals.telemetry.addLine(String.format("Motion: %d, %d", motion.x, motion.y));
            Globals.telemetry.addLine(String.format("Accumulated: %.1f, %.1f", opticalPose.position.x, opticalPose.position.y));
            Globals.telemetry.addLine("Time: " + opticalTimer.getSummary());

            // Draw the Mecanum robot pose:
            drive.updatePoseEstimate();
            Globals.canvas.setStroke("#3F51B5");
            MecanumDrive.drawRobot(Globals.canvas, drive.pose);

            Globals.endLoop();
        }

        TimeSplitter.reportAllResults();
    }
}
