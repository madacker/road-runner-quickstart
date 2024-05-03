package org.firstinspires.ftc.teamcode.sparkfun;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.SimpleMecanumDrive;

@TeleOp(group="Optical")
public class SimpleOpticalDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SimpleMecanumDrive drive = new SimpleMecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        double startYaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        waitForStart();

        int zeroCount = 0;

        while (opModeIsActive()) {
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);
            drive.setDrivePowers(powers);

            double imuHeading = Globals.normalizeAngle(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - startYaw);
            SparkFunOTOS.otos_pose2d_t pose = drive.optical.getPosition();
            if ((pose.x == 0) && (pose.y == 0) && (pose.h == 0)) {
                zeroCount++;
            }

            Pose2d pose2d = new Pose2d(pose.x, pose.y, pose.h);


            telemetry.addData("x", pose.x);
            telemetry.addData("y", pose.y);
            telemetry.addData("Sparkfun heading", "%.2f°", Math.toDegrees(pose.h));
            telemetry.addData("IMU heading", "%.2f°", Math.toDegrees(imuHeading));
            telemetry.addData("Heading deviation", "%.2f°", Math.toDegrees(imuHeading - pose.h));
            telemetry.addLine();
            telemetry.addData("Zero count", zeroCount);
            telemetry.update();

            // Begin drawing:
            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();

            // Draw the robot pose:
            c.setStroke("#3F51B5");
            MecanumDrive.drawRobot(c, pose2d);

            // Done drawing:
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(p);
        }

        TimeSplitter.reportAllResults();
    }
}
