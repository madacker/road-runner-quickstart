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

class AutoParker {
    // Target pose:
    final Pose2d target = new Pose2d(0, 0, 0);

    // Increase a velocity vector by a specified acceleration amount, clamping it to a range:
    Vector2d addVelocity(Vector2d velocity, double acceleration, double minSpeed, double maxSpeed) {
        double vectorMagnitude = Math.sqrt(velocity.sqrNorm());
        double speed = vectorMagnitude + acceleration;
        if (speed < minSpeed)
            speed = minSpeed;
        if (speed > maxSpeed)
            speed = maxSpeed;
        return velocity.times(speed / vectorMagnitude);
    }

    void park(MecanumDrive drive, TelemetryPacket packet, Canvas canvas) {
        // @@@ Need to figure out stopping

        // Calculate the vector from the current position to the target:
        Vector2d radialVector = target.position.minus(drive.pose.position);

        // Calculate distance to the target:
        double distanceToTarget = Math.sqrt(radialVector.sqrNorm());

        // Calculate the maximum speed directly towards the target assuming constant
        // deceleration. We use the kinematic equation "v^2 = u^2 + 2as" where v is the
        // current velocity, u is the initial velocity, a is the acceleration, s is distance
        // traveled. We apply it in reverse:
        double approachSpeed = Math.sqrt(2 * Math.abs(drive.PARAMS.minProfileAccel) * distanceToTarget);

        // Clamp the approach speed to the maximum speed:
        approachSpeed = Math.min(approachSpeed, drive.PARAMS.maxWheelVel);

        // Convert the velocity to be field-relative:
        PoseVelocity2d poseVelocity = drive.pose.times(drive.poseVelocity);

        // Compute the radial vector component of the current velocity (projection of
        // 'poseVelocity.linearVel' onto 'radialVector'):
        Vector2d radialVelocity = radialVector.times(
                poseVelocity.linearVel.dot(radialVector) / radialVector.sqrNorm());

        // Calculate the perpendicular velocity vector (velocity tangent to our target):
        Vector2d perpVelocity = poseVelocity.linearVel.minus(radialVelocity);

        // Subtract velocity from the perpendicular velocity vector and add it to the
        // radial vector:
        perpVelocity = addVelocity(perpVelocity, drive.PARAMS.minProfileAccel, 0, approachSpeed);
        radialVelocity = addVelocity(radialVelocity, drive.PARAMS.maxProfileAccel, -drive.PARAMS.maxWheelVel, approachSpeed);

        // Add the component vectors to get our new velocity vector:
        Vector2d driveVelocity = perpVelocity.plus(radialVelocity);

        // TODO: Supply acceleration to improve feedforward approximation
        // @@@ drive.setDrivePowers(null, new PoseVelocity2d(driveVelocity, 0), null);

        // Draw the perpendicular vector in red, the radial vector in green:
        canvas.setStroke("#FF0000");
        canvas.strokeLine(drive.pose.position.x, drive.pose.position.y, perpVelocity.x, perpVelocity.y);
        canvas.setStroke("#00FF00");
        canvas.strokeLine(drive.pose.position.x, drive.pose.position.y, radialVelocity.x, radialVelocity.y);
    }
}

class Wall {
    final double WALL_Y = 24;
    void setDrivePowers(MecanumDrive drive, PoseVelocity2d manualPower, TelemetryPacket packet, Canvas canvas) {
        double distanceToWall = WALL_Y - drive.pose.position.y;

        // Calculate the maximum speed directly towards the wall assuming constant
        // deceleration. We use the kinematic equation "v^2 = u^2 + 2as" where v is the
        // current velocity, u is the initial velocity, a is the acceleration, s is distance
        // traveled. Handle negative distances:
        double maxApproachSpeed = Math.signum(distanceToWall)
                * Math.sqrt(2 * Math.abs(drive.PARAMS.minProfileAccel * distanceToWall));

        // Convert the velocity to be field-relative:
        PoseVelocity2d poseVelocity = drive.pose.times(drive.poseVelocity);

        // Compute how much faster it's going toward the wall than it should:
        double excessApproachSpeed = Math.max(poseVelocity.linearVel.y - maxApproachSpeed, 0);

        // Counteract the excess approach speed:
        Vector2d counterVelocity = new Vector2d(0, -excessApproachSpeed);

        packet.put("distanceToWall", distanceToWall);
        packet.put("velocity", poseVelocity.linearVel.y);
        packet.put("maxApproachSpeed", maxApproachSpeed); // @@@
        packet.put("excessApproachSpeed", excessApproachSpeed); // @@@

        // drive.setDrivePowers(manualPower, null, null);
        // drive.setDrivePowers(manualPower, new PoseVelocity2d(new Vector2d(0, -2), 0), null);
        drive.setDrivePowers(manualPower, new PoseVelocity2d(counterVelocity, 0), null);

        // Draw the wall if it's been activated:
        if (excessApproachSpeed > 0) {
            canvas.setStroke("#0000FF");
            canvas.strokeLine(-72, WALL_Y, 72, WALL_Y);
        }
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
        AutoParker parker = new AutoParker();
        Wall wall = new Wall();
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

            PoseVelocity2d manualPower = new PoseVelocity2d(new Vector2d(
                    stickShaper(-gamepad1.left_stick_y), stickShaper(-gamepad1.left_stick_x)),
                    stickShaper(-gamepad1.right_stick_x));

            // park.park(drive, packet, canvas); // @@@

            if (gamepad1.a) {
                parker.park(drive, packet, canvas);
            } else {
                // drive.setDrivePowers(manualPower);
                wall.setDrivePowers(drive, manualPower, packet, canvas);
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

