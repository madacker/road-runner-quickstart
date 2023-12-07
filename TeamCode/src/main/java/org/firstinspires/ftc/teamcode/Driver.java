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

class AutoPark {
    // Target pose:
    final Pose2d target = new Pose2d(0, 0, 0);

    // Increase a velocity vector by a specified acceleration amount, capping it to a range:
    Vector2d addVelocity(Vector2d velocity, double acceleration, double minSpeed, double maxSpeed) {
        double vectorMagnitude = Math.sqrt(velocity.sqrNorm());
        double speed = vectorMagnitude + acceleration;
        if (speed < minSpeed)
            speed = minSpeed;
        if (speed > maxSpeed)
            speed = maxSpeed;
        return velocity.times(speed / vectorMagnitude);
    }

    void park(MecanumDrive drive, Canvas canvas) {
        // Calculate the vector from the current position to the target:
        Vector2d radialVector = target.position.minus(drive.pose.position);

        // Calculate distance to the target:
        double distanceToTarget = Math.sqrt(radialVector.sqrNorm());

        // Calculate the maximum speed directly towards the target assuming constant
        // deceleration:
        double approachSpeed = Math.sqrt(2*Math.abs(drive.PARAMS.minProfileAccel)*distanceToTarget);

        // Clamp the approach speed to the maximum speed:
        if (approachSpeed > drive.PARAMS.maxWheelVel)
            approachSpeed = drive.PARAMS.maxWheelVel;

        // Convert the velocity to be field-relative:
        PoseVelocity2d poseVelocity = drive.pose.times(drive.poseVelocity);

        // Compute the radial vector component of the current velocity (projection of
        // 'poseVelocity.linearVel' onto 'radialVector'):
        Vector2d radialVelocity = radialVector.times(poseVelocity.linearVel.dot(radialVector)
                                / radialVector.sqrNorm());

        // Calculate the perpendicular velocity vector (velocity tangent to our target):
        Vector2d perpVelocity = poseVelocity.linearVel.minus(radialVelocity);

        perpVelocity = addVelocity(
                perpVelocity,
                drive.PARAMS.minProfileAccel,
                0,
                approachSpeed);
        radialVelocity = addVelocity(
                radialVelocity,
                drive.PARAMS.maxProfileAccel,
                -drive.PARAMS.maxWheelVel,
                approachSpeed);

        // Add the component vectors to get our new velocity vector:
        Vector2d driveVelocity = perpVelocity.plus(radialVelocity);

        // @@@ Need to supply acceleration
        drive.setDrivePowers(null, new PoseVelocity2d(driveVelocity, 0), null);
    }
}

@TeleOp(name="Driver", group="Aardvark")
public class Driver extends LinearOpMode {
    // Provide more precision for slight stick movements:
    public double stickShaper(double stickValue) {
        return Math.signum(stickValue) * Math.pow(stickValue, 2.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TimeSplitter startupTime = TimeSplitter.create("Startup time");
        TimeSplitter loopTime = TimeSplitter.create("Loop time");
        boolean initializedPose = false;

        startupTime.startSplit();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Refiner refiner = new Refiner(hardwareMap);
        Led led = new Led(hardwareMap);
        AutoPark park = new AutoPark();
        startupTime.endSplit();

        waitForStart();

        while (opModeIsActive()) {
            loopTime.startSplit();

            // Update the telemetry pose and update the LED loop:
            drive.updatePoseEstimate();
            led.update();

            // Set up for visualizations:
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // Handle input:
            if (gamepad1.a) {
                park.park(drive, canvas);
            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                stickShaper(-gamepad1.left_stick_y),
                                stickShaper(-gamepad1.left_stick_x)
                        ),
                        stickShaper(-gamepad1.right_stick_x)
                ));
            }

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

