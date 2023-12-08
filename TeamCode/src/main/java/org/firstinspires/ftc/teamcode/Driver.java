package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    PoseVelocity2d previousVelocity; // Previous velocity
    Vector2d tangentVector;          // Normalized version of original tangent vector
    double radialSpeed;              // Inches/s, can be negative
    double tangentSpeed;             // Inches/s, can be negative

    AutoParker(MecanumDrive drive, Pose2d target) {
        this.drive = drive;
        this.target = target;

        // Remember the time:
        this.previousTime = Actions.now();

        // Radial vector towards the target:
        Vector2d radialVector = target.position.minus(drive.pose.position);

        // Tangent vector -90 degrees from radial:
        tangentVector = new Vector2d(radialVector.y, -radialVector.x);

        // Normalize the tangent:
        tangentVector = tangentVector.div(tangentVector.norm());

        // Convert the velocity to field-relative coordinates:
        PoseVelocity2d velocity = drive.pose.times(drive.poseVelocity);

        // Remember it:
        previousVelocity = velocity;

        // Compute the current speed:
        double speed = Math.hypot(velocity.linearVel.x, velocity.linearVel.y);

        // Compute the angle from the current velocity direction to the target:
        double theta = Math.atan2(target.position.y, target.position.x)
                     - Math.atan2(tangentVector.y, tangentVector.x);

        // Compute the component speeds:
        tangentSpeed = Math.sin(theta) * speed;
        radialSpeed = Math.cos(theta) * speed;

        // If the radial velocity is current away from the robot, negate it:
        if (theta > Math.PI)
            theta -= 2*Math.PI;
        if (Math.abs(theta) > Math.PI/2)
            radialSpeed = -radialSpeed;
    }

    // Returns true when parked, false when not parked yet:
    boolean park(TelemetryPacket packet, Canvas canvas) {
        double now = Actions.now();
        double deltaT = now - previousTime;

        // Radial vector towards the target:
        Vector2d radialVector = target.position.minus(drive.pose.position);
        double radialDistance = radialVector.norm();

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
        double approachSpeed = Math.sqrt(2 * Math.abs(drive.PARAMS.minProfileAccel) * radialDistance);

        // Set the new radial speed as the minimum of the three:
        radialSpeed = Math.min(Math.min(increasingRadialSpeed, maxRadialSpeed), approachSpeed);

        Vector2d radialVelocity = radialVector.div(radialVector.norm()).times(radialSpeed);
        Vector2d tangentVelocity = tangentVector.times(tangentSpeed);

        // Add the component velocities to get our new velocity:
        PoseVelocity2d velocity = new PoseVelocity2d(radialVelocity.plus(tangentVelocity), 0);

        // Compute the new accelerations from the velocities:
        PoseVelocity2d acceleration = new PoseVelocity2d(new Vector2d(
                velocity.linearVel.x - previousVelocity.linearVel.x,
                velocity.linearVel.y - previousVelocity.linearVel.y),
                velocity.angVel - previousVelocity.angVel);

        // @@@ drive.setDrivePowers(null, velocity, acceleration);

        // Remember stuff for the next iteration:
        previousVelocity = velocity;
        previousTime = now;

        // Draw the perpendicular vector in red, the radial vector in green:
        canvas.setStrokeWidth(1);
        canvas.setStroke("#FF0000");
        canvas.strokeLine(drive.pose.position.x, drive.pose.position.y, tangentVelocity.x, tangentVelocity.y);

        canvas.setStroke("#00FF00");
        canvas.strokeLine(drive.pose.position.x, drive.pose.position.y, radialVelocity.x, radialVelocity.y);

        packet.put("tangentSpeed", tangentSpeed);
        packet.put("radialSpeed", radialSpeed);

        return false; // @@@
    }
}

//class AutoBackdrop extends AutoParker {
//    AutoBackdrop(MecanumDrive drive, Pose2d offsetPose, Pose2d placementPose) {
//        super(drive, offsetPose);
//    }
//
//    boolean iterate(TelemetryPacket packet, Canvas canvas) {
//        return super.park(packet, canvas);
//    }
//}

class AutoParkerOld {
    // Target pose:
    final Pose2d target = new Pose2d(0, 0, 0);

    // Our current velocity:
    PoseVelocity2d velocity;

    // @@@ Debugging only:
    boolean busted = false;

    // Increase a velocity vector by a specified acceleration amount, clamping it to a range:
    Vector2d addVelocity(Vector2d direction, Vector2d velocity, double acceleration, double minSpeed, double maxSpeed) {
        double speed = velocity.norm() + acceleration;
        if (speed < minSpeed)
            speed = minSpeed;
        if (speed > maxSpeed)
            speed = maxSpeed;
        double directionLength = direction.norm();
        if (directionLength == 0)
            return new Vector2d(0, 0);
        else
            return direction.div(directionLength).times(speed);
    }

    void dontPark() {
        velocity = null;
    }

    void park(MecanumDrive drive, double deltaT, TelemetryPacket packet, Canvas canvas) {
        // @@@ Need to figure out stopping
        if (busted)
            return; // @@@

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

        // When starting to park, inherit the velocity vector from the driver. Convert it
        // to be field-relative:
        if (velocity == null) {
            velocity = drive.pose.times(drive.poseVelocity);
        }

        // Compute the radial vector component of the current velocity (projection of
        // 'poseVelocity.linearVel' onto 'radialVector'):
        Vector2d radialVelocity = radialVector.times(
                velocity.linearVel.dot(radialVector) / radialVector.sqrNorm());

        // Calculate the perpendicular velocity vector (velocity tangent to our target):
        Vector2d perpVelocity = velocity.linearVel.minus(radialVelocity);

        // Subtract velocity from the perpendicular velocity vector and add it to the
        // radial vector:
        perpVelocity = addVelocity(perpVelocity, perpVelocity, drive.PARAMS.minProfileAccel * deltaT, 0, approachSpeed);

        // @@@ Fix radial direction?
        radialVelocity = addVelocity(radialVector, radialVelocity, drive.PARAMS.maxProfileAccel * deltaT, -drive.PARAMS.maxWheelVel, approachSpeed);

        packet.put("distanceToTarget", distanceToTarget);
        packet.put("approachSpeed", approachSpeed);
        packet.put("radialVelocity", radialVelocity.norm());
        packet.put("deltaT", deltaT);

        // Add the component vectors to get our new velocity vector:
        PoseVelocity2d thisVelocity = new PoseVelocity2d(perpVelocity.plus(radialVelocity), 0);

        // TODO: Supply acceleration to improve feedforward approximation
        drive.setDrivePowers(null, thisVelocity, null);

        // Draw the perpendicular vector in red, the radial vector in green:
        canvas.setStrokeWidth(1);
        canvas.setStroke("#FF0000");
        if (!Double.isNaN(perpVelocity.x) && !Double.isNaN(perpVelocity.y))
            canvas.strokeLine(drive.pose.position.x, drive.pose.position.y, perpVelocity.x, perpVelocity.y);
        else
            busted = true;

        canvas.setStroke("#00FF00");
        if (!Double.isNaN(radialVelocity.x) && !Double.isNaN(radialVelocity.y))
            canvas.strokeLine(drive.pose.position.x, drive.pose.position.y, radialVelocity.x, radialVelocity.y);
        else
            busted = true;

        if (!Double.isNaN(thisVelocity.linearVel.x) && !Double.isNaN(thisVelocity.linearVel.y))
            velocity = thisVelocity;
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
        startupTime.endSplit();
        double lastTime = Actions.now();
        AutoParker parker = null;

        waitForStart();

        while (opModeIsActive()) {

            loopTime.startSplit();

            // Update the telemetry pose and update the LED loop:
            drive.updatePoseEstimate();
            led.update();

            // Set up for visualizations:
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // Calculate delta-t:
            double newTime = Actions.now();
            double deltaT = newTime - lastTime;
            lastTime = newTime;

            // Handle input:
            PoseVelocity2d manualPower = new PoseVelocity2d(new Vector2d(
                    stickShaper(-gamepad1.left_stick_y), stickShaper(-gamepad1.left_stick_x)),
                    stickShaper(-gamepad1.right_stick_x));

            boolean onAuto = false;
            if ((gamepad1.a) && (parker == null))
                parker = new AutoParker(drive, new Pose2d(0, 0, 0));
            if (parker != null) {
                onAuto = parker.park(packet, canvas);
                if (!onAuto)
                    parker = null;
            }
            // @@@ Check if 'onAuto'
            drive.setDrivePowers(manualPower);

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

