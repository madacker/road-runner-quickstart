package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

class AutoParker {
    MecanumDrive drive;              // Used to get pose and poseVelocity and set the motors
    Pose2d target;                   // Target pose
    double previousTime;             // Previous time in seconds
    Vector2d tangentVector;          // Normalized version of original tangent vector
    double radialSpeed;              // Inches/s, can be negative
    double tangentSpeed;             // Inches/s, can be negative
    double angularSpeed;             // Radians/s, can be negative

    double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2*Math.PI;
        while (angle < -Math.PI)
            angle += 2*Math.PI;
        return angle;
    }

    AutoParker(MecanumDrive drive, TelemetryPacket packet, Pose2d target) {
        this.drive = drive;
        this.target = target;

        // Remember the time:
        this.previousTime = Actions.now();

        // Position --------------------------------------------------------------------------------

        // Radial vector towards the target:
        Vector2d radialVector = target.position.minus(drive.pose.position);

        // Tangent vector -90 degrees from radial:
        tangentVector = new Vector2d(radialVector.y, -radialVector.x);

        // Normalize the tangent, being careful to protect against divide-by-zero:
        double tangentLength = tangentVector.norm();
        if (tangentLength != 0)
            tangentVector = tangentVector.div(tangentLength);

        // Convert the velocity to field-relative coordinates:
        PoseVelocity2d velocity = drive.pose.times(drive.poseVelocity);

        // Compute the current speed:
        double speed = Math.hypot(velocity.linearVel.x, velocity.linearVel.y);

        // Compute the angle from the robot's current direction to the target:
        double theta = Math.atan2(radialVector.y, radialVector.x)
                     - Math.atan2(velocity.linearVel.y, velocity.linearVel.x);

        // Compute the component speeds:
        tangentSpeed = Math.sin(theta) * speed;
        radialSpeed = Math.cos(theta) * speed;

        // Handle heading:
        angularSpeed = velocity.angVel;
    }

    // Returns true when parked, false when not parked yet:
    boolean park(TelemetryPacket packet) {
        // Radial vector towards the target:
        Vector2d radialVector = target.position.minus(drive.pose.position);
        double radialLength = radialVector.norm();

        // Angular distance to the target:
        double angularDelta = normalizeAngle(target.heading.log() - drive.pose.heading.log());

        // We're done if the distance is small enough!
        if ((radialLength < 0.5) && (Math.abs(angularDelta) < Math.toRadians(2)))
            return false;

        double now = Actions.now();
        double deltaT = now - previousTime;

        // Position --------------------------------------------------------------------------------

        // Decrease the magnitude of the tangent speed with a minimum value of zero:
        double tangentMagnitude = Math.max(0,
                Math.abs(tangentSpeed) + drive.PARAMS.minProfileAccel * deltaT);

        // Convert to the signed tangent speed:
        tangentSpeed = Math.signum(tangentSpeed) * tangentMagnitude;

        // Increase the speed towards the target:
        double increasingRadialSpeed = radialSpeed + drive.PARAMS.maxProfileAccel * deltaT;

        // Compute the maximum possible speed towards the target, accounting for the speed along
        // the tangent:
        double maxRadialSpeed = Math.hypot(tangentSpeed, drive.PARAMS.maxWheelVel);

        // Calculate the maximum speed directly towards the target assuming constant
        // deceleration. We use the kinematic equation "v^2 = u^2 + 2as" where v is the
        // current velocity, u is the initial velocity, a is the acceleration, s is distance
        // traveled. We apply it in reverse:
        double radialApproach = Math.sqrt(2 * Math.abs(drive.PARAMS.minProfileAccel) * radialLength);

        // Set the new radial speed as the minimum of the three:
        radialSpeed = Math.min(Math.min(increasingRadialSpeed, maxRadialSpeed), radialApproach);

        Vector2d radialVelocity = radialVector.div(radialLength).times(radialSpeed);
        Vector2d tangentVelocity = tangentVector.times(tangentSpeed);

        // Heading ---------------------------------------------------------------------------------

        // Calculate the angular velocity as a positive magnitude:
        if (angularDelta < 0) {
            double increasingAngularSpeed = angularSpeed - drive.PARAMS.maxAngAccel * deltaT;
            double maxAngularSpeed = -drive.PARAMS.maxAngVel;
            double angularApproach = -Math.sqrt(2 * drive.PARAMS.maxAngAccel * Math.abs(angularDelta));
            angularSpeed = Math.max(Math.max(increasingAngularSpeed, maxAngularSpeed), angularApproach);
        } else {
            double increasingAngularSpeed = angularSpeed + drive.PARAMS.maxAngAccel * deltaT;
            double maxAngularSpeed = drive.PARAMS.maxAngVel;
            double angularApproach = Math.sqrt(2 * drive.PARAMS.maxAngAccel * Math.abs(angularDelta));
            angularSpeed = Math.min(Math.min(increasingAngularSpeed, maxAngularSpeed), angularApproach);
        }

        // Combine ---------------------------------------------------------------------------------

        // Add the component velocities to get our new velocity:
        PoseVelocity2d velocity = new PoseVelocity2d(radialVelocity.plus(tangentVelocity), angularSpeed);
        drive.setDrivePowers(null, velocity);

        // Remember stuff for the next iteration:
        previousTime = now;

//        Canvas canvas = packet.fieldOverlay();
//
//        // Draw the perpendicular vector in red, the radial vector in green:
//        canvas.setStrokeWidth(1);
//        canvas.setStroke("#FF0000");
//        canvas.strokeLine(
//                drive.pose.position.x,
//                drive.pose.position.y,
//                drive.pose.position.x + tangentVelocity.x,
//                drive.pose.position.y + tangentVelocity.y);
//
//        canvas.setStroke("#00FF00");
//        canvas.strokeLine(
//                drive.pose.position.x,
//                drive.pose.position.y,
//                drive.pose.position.x + radialVelocity.x,
//                drive.pose.position.y + radialVelocity.y);

        return true;
    }
}

class Wall {
    MecanumDrive drive;
    Vector2d wallPoint;
    Vector2d wallVector;

    // The wall will repulse any robot approaching from the left of the vector.
    Wall(MecanumDrive drive, Vector2d point, Vector2d vector) {
        this.drive = drive;
        wallPoint = point;
        wallVector = vector;
    }

    // The velocity must be field-relative, calibrated inches/s and radians/s:
    PoseVelocity2d repulse(PoseVelocity2d velocity, TelemetryPacket packet) {

        // Length of the velocity vector:
        double velocityMagnitude = Math.hypot(velocity.linearVel.x, velocity.linearVel.y);
        if (velocityMagnitude == 0)
            return velocity;

        // Direction that's normal to the wall, pointing towards the wall from the robot's
        // perspective:
        double normalAngle = Math.atan2(wallVector.y, wallVector.x) - Math.PI/2;

        // Direction of the velocity vector:
        double velocityAngle = Math.atan2(velocity.linearVel.y, velocity.linearVel.x);

        // Break the velocity vector into constituent velocities towards and along the wall:
        double towardWallAngle = velocityAngle - normalAngle;
        double towardWallVelocity = Math.cos(towardWallAngle) * velocityMagnitude;
        double alongWallVelocity = Math.sin(towardWallAngle) * velocityMagnitude;

        // Compute the distance to the wall:
        double dx = wallPoint.x - drive.pose.position.x;
        double dy = wallPoint.y - drive.pose.position.y;
        double pointToPointDistance = Math.hypot(dy, dx);
        double pointToPointAngle = Math.atan2(dy, dx);
        double distance = Math.cos(pointToPointAngle - towardWallAngle) * pointToPointDistance;

packet.put("Distance to wall", distance);
packet.put("Wall approach velocity", towardWallVelocity);

        // Calculate the maximum speed directly towards the target assuming constant
        // deceleration. We use the kinematic equation "v^2 = u^2 + 2as" where v is the
        // current velocity, u is the initial velocity, a is the acceleration, s is distance
        // traveled. We apply it in reverse:
        double maxApproachVelocity = Math.sqrt(2 * Math.abs(drive.PARAMS.minProfileAccel) * distance);
        towardWallVelocity = Math.min(towardWallVelocity, maxApproachVelocity);

packet.put("Max approach", maxApproachVelocity);

        // Constitute the revised velocity's component vectors:
        Vector2d towardWallVector = new Vector2d(
                Math.cos(normalAngle) * towardWallVelocity,
                Math.sin(normalAngle) * towardWallVelocity);
        Vector2d alongWallVector = new Vector2d(
                Math.cos(normalAngle + Math.PI/2) * alongWallVelocity,
                Math.sin(normalAngle + Math.PI/2) * alongWallVelocity);

        // Return the new velocity's aggregate result:
        return new PoseVelocity2d(new Vector2d(
                towardWallVector.x + alongWallVector.x,
                towardWallVector.y + alongWallVector.y),
                velocity.angVel);
    }
}

@TeleOp(name="Driver", group="Aardvark")
public class Driver extends LinearOpMode {
    // Shape the stick input for more precision at slow speeds:
    public double shapeStick(double stickValue) {
        return Math.signum(stickValue) * Math.pow(stickValue, 2.0);
    }

    // Scale the stick input to a shaped result that accounts for the dead zone:
    public double scaleStick(double stickValue, double scale) {
        final double DEAD_ZONE = 0.15; // Assume 15%, not forgetting that the range is [-1, 1]
        if (Math.abs(stickValue) <= DEAD_ZONE)
            return 0;

        // Shape the result:
        stickValue = shapeStick(stickValue);
        // Push away from the dead zone:
        stickValue = (1.0 - DEAD_ZONE) * stickValue + Math.signum(stickValue) * DEAD_ZONE;
        // Return the scaled result:
        return stickValue * scale;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TimeSplitter loopTime = TimeSplitter.create("> Loop");

        double maxLinearAcceleration = 0;
        double minLinearDeceleration = 0;
        double maxLinearSpeed = 0;
        double previousLinearSpeed = 0;
        double maxAngularAcceleration = 0;
        double minAngularDeceleration = 0;
        double maxAngularSpeed = 0;
        double previousAngularSpeed = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Refiner refiner = new Refiner(hardwareMap);
        Led led = new Led(hardwareMap);
        AutoParker parker = null;
        Wall wall = new Wall(drive, new Vector2d(-72, -36), new Vector2d(24, -24));

        // Feed forward model: voltage = kS + kV*velocityInTicksPerSecond + kA*acceleration
        double fullAxialSpeed
                = ((drive.voltageSensor.getVoltage() - drive.PARAMS.kS) / drive.PARAMS.kV)
                * drive.PARAMS.inPerTick;
        double fullLateralSpeed
                = fullAxialSpeed * drive.PARAMS.lateralInPerTick / drive.PARAMS.inPerTick;
        // Rotations/s = fullAxialSpeed / (pi * wheelbase)
        // Radians/s = 2 * pi * rotations/s
        double fullAngularSpeed
                = 2 * fullAxialSpeed / (drive.PARAMS.trackWidthTicks * drive.PARAMS.inPerTick);

        waitForStart();

        while (opModeIsActive()) {
            loopTime.startSplit();

            // Update the telemetry pose and update the LED loop:
            drive.updatePoseEstimate();
            led.update();

            // Set up for visualizations:
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // The 'A' button activates the auto-parker:
            boolean parkingActivated = false;
            if (!gamepad1.a)
                parker = null;
            else {
                if (parker == null)
                    parker = new AutoParker(drive, packet, new Pose2d(0, 0, 0));
                parkingActivated = parker.park(packet);
            }

            // The 'left-bumper' button activates Road Runner homing:
            boolean roadrunnerActivated = drive.doActionsWork(packet);
            if (!gamepad1.left_bumper)
                drive.abortActions();
            else if (!roadrunnerActivated) {
                // Ensure that velocity is zero-ish:
                if ((Math.abs(drive.poseVelocity.linearVel.x) < 0.1) &&
                    (Math.abs(drive.poseVelocity.linearVel.y) < 0.1) &&
                    (Math.abs(drive.poseVelocity.angVel) < 0.1)) {

                    boolean toBackdrop = drive.pose.position.x < 0.0;
                    Action action;

                    if (toBackdrop) {
                        // Go to the blue backdrop:
                        Pose2d keyPose = new Pose2d(-36, -36, Math.PI);
                        double startTangent = Math.atan2(
                                keyPose.position.y - drive.pose.position.y,
                                keyPose.position.x - drive.pose.position.x);
                        action = drive.actionBuilder(drive.pose)
                                .setTangent(startTangent)
                                .splineToSplineHeading(keyPose, 0)
                                .splineTo(new Vector2d(12, -36), 0)
                                .splineTo(new Vector2d(48, 36), 0)
                                .build();
                    } else {
                        // Go to the blue wing:
                        Pose2d keyPose = new Pose2d(48, 36, Math.PI);
                        double startTangent = Math.atan2(
                                keyPose.position.y - drive.pose.position.y,
                                keyPose.position.x - drive.pose.position.x);
                        action = drive.actionBuilder(drive.pose)
                                .setTangent(startTangent)
                                .splineToSplineHeading(new Pose2d(12, -36, Math.PI), Math.PI)
                                .splineTo(new Vector2d(-36, -36), Math.PI)
                                .splineTo(new Vector2d(-71 + 18, -72 + 18), Math.toRadians(225))
                                .build();
                    }
                    drive.runParallel(action);
                    roadrunnerActivated = true;
                }
            }

            // Manually drive:
            if ((!parkingActivated) && (!roadrunnerActivated)) {
                if (true) {
                    PoseVelocity2d calibratedVelocity = new PoseVelocity2d(new Vector2d(
                            scaleStick(-gamepad1.left_stick_y, fullAxialSpeed),
                            scaleStick(-gamepad1.left_stick_x, fullLateralSpeed)),
                            scaleStick(-gamepad1.right_stick_x, fullAngularSpeed));

                    PoseVelocity2d fieldVelocity = drive.pose.times(calibratedVelocity);

                    wall.repulse(fieldVelocity, packet); // @@@

                    drive.setDrivePowers(null, fieldVelocity);
                } else {
                    PoseVelocity2d manualPower = new PoseVelocity2d(new Vector2d(
                            shapeStick(-gamepad1.left_stick_y),
                            shapeStick(-gamepad1.left_stick_x)),
                            shapeStick(-gamepad1.right_stick_x));
                    drive.setDrivePowers(manualPower);
                }
            }

            // Refine the pose estimate using AprilTags:
            Pose2d refinedPose = refiner.refinePose(drive.pose, canvas);
            if (refinedPose != null) {
                led.setSteadyColor(Led.Color.GREEN);
                led.setPulseColor(Led.Color.RED, 0.25);

                // Set the new pose and record it:
                drive.pose = refinedPose;
                drive.recordPose(refinedPose, 1);
            }

            // Draw the pose history:
            drive.drawPoseHistory(canvas);

            // Draw the refinement history:

            // Draw the best estimate pose:
            canvas.setStroke("#3F51B5");
            MecanumDrive.drawRobot(canvas, drive.pose);

            // Log interesting data:
            double linearSpeed = Math.hypot(drive.poseVelocity.linearVel.x, drive.poseVelocity.linearVel.y);
            double linearSpeedDelta = linearSpeed - previousLinearSpeed;
            if (linearSpeedDelta > 0) {
                maxLinearAcceleration = Math.max(linearSpeedDelta, maxLinearAcceleration);
            } else {
                minLinearDeceleration = Math.min(linearSpeedDelta, minLinearDeceleration);
            }
            previousLinearSpeed = linearSpeed;
            maxLinearSpeed = Math.max(linearSpeed, maxLinearSpeed);

            double angularSpeed = Math.abs(drive.poseVelocity.angVel);
            double angularSpeedDelta = angularSpeed - previousAngularSpeed;
            if (angularSpeedDelta > 0) {
                maxAngularAcceleration = Math.max(angularSpeedDelta, maxAngularAcceleration);
            } else {
                minAngularDeceleration = Math.min(angularSpeedDelta, minAngularDeceleration);
            }
            previousAngularSpeed = angularSpeed;
            maxAngularSpeed = Math.max(angularSpeed, maxAngularSpeed);

//            packet.put("Linear speed", linearSpeed);
//            packet.put("Linear delta", linearSpeedDelta);
//            packet.put("Linear theoretical", fullAxialSpeed);
//            packet.put("Angular speed", angularSpeed);
//            packet.put("Angular delta", angularSpeedDelta);
//            packet.put("Angular theoretical", fullAngularSpeed);

            // Finish up:
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            loopTime.endSplit();
        }

        // Cleanup:
        refiner.close();
        TimeSplitter.logAllResults();
        led.setSteadyColor(Led.Color.OFF);

        // Output summary:
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine(String.format("Linear: top-speed: %.1f, theoretical: %.1f, accel: %.1f, decel: %.1f",
                maxLinearSpeed, fullAxialSpeed, maxLinearAcceleration, minLinearDeceleration));
        packet.addLine(String.format("Angular: top-speed: %.2f, accel: %.2f, decel: %.2f",
                maxAngularSpeed, maxAngularAcceleration, minAngularDeceleration));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

