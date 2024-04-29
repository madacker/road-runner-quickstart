package org.firstinspires.ftc.teamcode.explorations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sparkfun.SparkFunOTOS;

@TeleOp
public class OdometrySensorExample extends LinearOpMode
{
    // Define sensor
    SparkFunOTOS myOdometrySensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get sensor from hardware map
        System.out.println(">> Calling get...\n");
        myOdometrySensor = hardwareMap.get(SparkFunOTOS.class, "sparkfun");

        // Set offset location of sensor relative to center of robot
        SparkFunOTOS.otos_pose2d_t offsetPose = myOdometrySensor.new otos_pose2d_t(3.7, -.55, 0);
        System.out.println(">> Calling setOffset...\n");
        myOdometrySensor.setOffset(offsetPose);

        // Set scalar correction factors for translation and rotation
        System.out.println(">> Calling setLinearScalar...\n");
        myOdometrySensor.setLinearScalar(1.0);
        System.out.println(">> Calling setAngularScalar...\n");
        myOdometrySensor.setAngularScalar(1.0);

        // Calibrate IMU. Must be stationary and flat during this time! By default, this will take
        // about 612ms, but can be made faster by taking fewer samples
        System.out.println(">> Calling calibrateImu...\n");
        myOdometrySensor.calibrateImu();

        // Reset tracking algorithm on OTOS
        System.out.println(">> Calling resetTracking...\n");
        myOdometrySensor.resetTracking();

        // Set known location on field
        SparkFunOTOS.otos_pose2d_t newPose = myOdometrySensor.new otos_pose2d_t(5, 10, 45);
        System.out.println(">> Calling setPosition...\n");
        myOdometrySensor.setPosition(newPose);

        waitForStart();

        while (opModeIsActive())
        {
            // System.out.println(">> Calling getPosition...\n");
            SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();

            telemetry.addData("X (inch)", pose.x);
            telemetry.addData("Y (inch)", pose.y);
            telemetry.addData("H (Deg)", pose.h);
            telemetry.update();
        }
    }
}
