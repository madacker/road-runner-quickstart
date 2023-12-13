package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

class AutoParker {
    MecanumDrive drive;              // Used to get pose and poseVelocity and set the motors
    Pose2d target;                   // Target pose
    double previousTime;             // Previous time in seconds
    PoseVelocity2d previousVelocity; // Previous velocity
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

        // Remember it:
        previousVelocity = velocity;

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

        packet.put("initial angularSpeed", Math.toDegrees(angularSpeed));
        packet.put("initial angle", normalizeAngle(target.heading.log() - drive.pose.heading.log()));
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

        // Compute the new accelerations from the velocities:
        PoseVelocity2d acceleration = new PoseVelocity2d(new Vector2d(
                velocity.linearVel.x - previousVelocity.linearVel.x,
                velocity.linearVel.y - previousVelocity.linearVel.y),
                velocity.angVel - previousVelocity.angVel);

        drive.setDrivePowers(null, velocity, acceleration);

        // Remember stuff for the next iteration:
        previousVelocity = velocity;
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

@TeleOp(name="Driver", group="Aardvark")
public class Driver extends LinearOpMode {
    // Provide more precision for slight stick movements:
    public double stickShaper(double stickValue) {
        return Math.signum(stickValue) * Math.pow(stickValue, 2.0);
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

        // Feed forward model: voltage = kS + kV*velocityInTicksPerSecond + kA*acceleration
        double maxTheoreticalSpeed
                = ((drive.voltageSensor.getVoltage() - drive.PARAMS.kS) / drive.PARAMS.kV)
                * drive.PARAMS.inPerTick;

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
            boolean autoActivated = false;
            if (!gamepad1.a)
                parker = null;
            else {
                if (parker == null)
                    parker = new AutoParker(drive, packet, new Pose2d(0, 0, 0));
                autoActivated = parker.park(packet);
            }

            // Manually drive:
            if (!autoActivated) {
                if (false) {
                    double lateralFactor = drive.PARAMS.lateralInPerTick / drive.PARAMS.inPerTick;
                    PoseVelocity2d manualVelocity = new PoseVelocity2d(new Vector2d(
                            stickShaper(-gamepad1.left_stick_y) * drive.PARAMS.maxWheelVel,
                            stickShaper(-gamepad1.left_stick_x) * drive.PARAMS.maxWheelVel * lateralFactor),
                            stickShaper(-gamepad1.right_stick_x) * drive.PARAMS.maxAngVel);

                    // @@@ Calculate acceleration:
                    drive.setDrivePowers(null, manualVelocity, null);
                } else {
                    PoseVelocity2d manualPower = new PoseVelocity2d(new Vector2d(
                            stickShaper(-gamepad1.left_stick_y),
                            stickShaper(-gamepad1.left_stick_x)),
                            stickShaper(-gamepad1.right_stick_x));
                    drive.setDrivePowers(manualPower);
                }
            }

            // Draw AprilTag poses and refine them:
            Pose2d refinedPose = refiner.refinePose(drive.pose, canvas);
            if (refinedPose != null) {
                led.setSteadyColor(Led.Color.GREEN);
                led.setPulseColor(Led.Color.RED, 0.25);
                drive.pose = refinedPose;
            }

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
            maxAngularSpeed = Math.max(angularSpeedDelta, maxAngularSpeed);

            packet.put("Linear speed", linearSpeed);
            packet.put("Linear delta", linearSpeedDelta);
            packet.put("Linear theoretical", maxTheoreticalSpeed);
            packet.put("Angular speed", angularSpeed);
            packet.put("Angular delta", angularSpeedDelta);

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
                maxLinearSpeed, maxTheoreticalSpeed, maxLinearAcceleration, minLinearDeceleration));
        packet.addLine(String.format("Angular: top-speed: %.2f, accel: %.2f, decel: %.2f",
                maxAngularSpeed, maxAngularAcceleration, minAngularDeceleration));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

