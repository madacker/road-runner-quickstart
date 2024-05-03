package org.firstinspires.ftc.teamcode.sparkfun;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;

@TeleOp(group="Optical")
public class OdometrySensorPrinter extends LinearOpMode
{
    // Define sensor
    SparkFunOTOS myOdometrySensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get sensor from hardware map
        myOdometrySensor = hardwareMap.get(SparkFunOTOS.class, "sparkfun");

        // Set offset location of sensor relative to center of robot
        SparkFunOTOS.otos_pose2d_t offsetPose = new SparkFunOTOS.otos_pose2d_t(3.7, -.55, 0);
        myOdometrySensor.setOffset(offsetPose);

        // Set scalar correction factors for translation and rotation
        myOdometrySensor.setLinearScalar(1.0);
        myOdometrySensor.setAngularScalar(1.0);

        // Calibrate IMU. Must be stationary and flat during this time! By default, this will take
        // about 612ms, but can be made faster by taking fewer samples
        myOdometrySensor.calibrateImu();

        // Reset tracking algorithm on OTOS
        myOdometrySensor.resetTracking();

        // Set known location on field
        SparkFunOTOS.otos_pose2d_t newPose = new SparkFunOTOS.otos_pose2d_t(5, 10, 45);
        myOdometrySensor.setPosition(newPose);

        waitForStart();

        TimeSplitter allSlow = TimeSplitter.create("All slow");
        TimeSplitter allFast = TimeSplitter.create("All fast");

        while (opModeIsActive())
        {
            allSlow.startSplit();
            SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();
            SparkFunOTOS.otos_pose2d_t velocity = myOdometrySensor.getVelocity();
            SparkFunOTOS.otos_pose2d_t acceleration = myOdometrySensor.getAcceleration();
            allSlow.endSplit();

            allFast.startSplit();
            SparkFunOTOS.otos_all_poses_t all = myOdometrySensor.getAllPoses();
            allFast.endSplit();

            telemetry.addData("Position X (inch)", "%.2f/%.2f", pose.x, all.position.x);
            telemetry.addData("Position Y (inch)", "%.2f/%.2f", pose.y, all.position.y);
            telemetry.addData("Position H (Deg)", "%.2f/%.2f", pose.h, all.position.h);
            telemetry.addLine();

            telemetry.addData("Velocity X (inch)", "%.2f/%.2f", velocity.x, all.velocity.x);
            telemetry.addData("Velocity Y (inch)", "%.2f/%.2f", velocity.y, all.velocity.y);
            telemetry.addData("Velocity H (Deg/s)", "%.2f/%.2f", velocity.h, all.velocity.h);
            telemetry.addLine();

            telemetry.addData("Acceleration X (inch)", "%.2f/%.2f", acceleration.x, all.acceleration.x);
            telemetry.addData("Acceleration Y (inch)", "%.2f/%.2f", acceleration.y, all.acceleration.y);
            telemetry.addData("Acceleration H (Deg/s/s)", "%.2f/%.2f", acceleration.h, all.acceleration.h);

            telemetry.update();
        }

        TimeSplitter.reportAllResults();
    }
}
