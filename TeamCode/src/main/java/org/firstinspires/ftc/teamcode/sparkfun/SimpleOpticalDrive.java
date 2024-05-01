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

@TeleOp(name="SimpleOpticalDrive",group="Explore")
public class SimpleOpticalDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Settings settings = new Settings(telemetry, gamepad1);
        Stats stats = new Stats();
        Globals globals = new Globals(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), globals);
        SparkFunOTOS optical = hardwareMap.get(SparkFunOTOS.class, "sparkfun");

        // Configure the Spark Fun sensor:
        optical.setOffset(new SparkFunOTOS.otos_pose2d_t(5.56, 3.39, 179.9));
        optical.setAngularUnit(SparkFunOTOS.otos_angular_unit_t.kOtosAngularUnitRadians);
        optical.setLinearScalar(0.956);
        optical.setAngularScalar(1.0);
        optical.calibrateImu();
        optical.resetTracking(); // Seems to be needed?

        waitForStart();

        while (opModeIsActive()) {
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);
            drive.setDrivePowers(powers);

            SparkFunOTOS.otos_pose2d_t pose = optical.getPosition();

            Pose2d pose2d = new Pose2d(pose.x, pose.y, pose.h);
            telemetry.addData("x", pose.x);
            telemetry.addData("y", pose.y);
            telemetry.addData("heading (°)", Math.toDegrees(pose.h));
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
