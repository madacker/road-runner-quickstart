package org.firstinspires.ftc.teamcode.explorations;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.opticaltracking.OpticalTrackingPaa5100;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="OpticalTrackingTest",group="Explore")
public class OpticalTrackingTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.initialize();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        OpticalTrackingPaa5100 optical = hardwareMap.get(OpticalTrackingPaa5100.class, "optical2");

        waitForStart();

        while (opModeIsActive()) {
            Globals.startLoop(telemetry);

            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);

            drive.setDrivePowers(powers);
            drive.updatePoseEstimate();

            // Draw the Mecanum robot pose:
            Globals.canvas.setStroke("#3F51B5");
            MecanumDrive.drawRobot(Globals.canvas, drive.pose);

            Globals.endLoop();
        }
    }
}
