package org.firstinspires.ftc.teamcode.sparkfun;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.Stats;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.SimpleMecanumDrive;

@TeleOp(name="SimpleOpticalDrive",group="Explore")
public class SimpleOpticalDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Settings settings = new Settings(telemetry, gamepad1);
        Stats stats = new Stats();
        Globals globals = new Globals(hardwareMap, telemetry);

        SimpleMecanumDrive drive = new SimpleMecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        double startYaw = Globals.getRawYaw();

        waitForStart();

        while (opModeIsActive()) {
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);
            drive.setDrivePowers(powers);

            double imuHeading = Globals.normalizeAngle(Globals.getRawYaw() - startYaw);
            SparkFunOTOS.otos_pose2d_t pose = drive.optical.getPosition();

            Pose2d pose2d = new Pose2d(pose.x, pose.y, pose.h);
            telemetry.addData("x", pose.x);
            telemetry.addData("y", pose.y);
            telemetry.addData("Sparkfun heading", "%.2f°", Math.toDegrees(pose.h));
            telemetry.addData("IMU heading", "%.2f°", Math.toDegrees(imuHeading));
            telemetry.addData("Heading deviation", "%.2f°", Math.toDegrees(imuHeading - pose.h));
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
