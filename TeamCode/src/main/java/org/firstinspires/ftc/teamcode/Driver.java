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

    TimeSplitter parkTime = TimeSplitter.create("> Park");
    TimeSplitter parkPowerTime = TimeSplitter.create("> Park Power");

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

        // Compute the angle from the robot's current direction to the target:
        double theta = Math.atan2(radialVector.y, radialVector.x)
                     - Math.atan2(velocity.linearVel.y, velocity.linearVel.x);

        // Compute the component speeds:
        tangentSpeed = Math.sin(theta) * speed;
        radialSpeed = Math.cos(theta) * speed;
    }

    // Returns true when parked, false when not parked yet:
    boolean park(TelemetryPacket packet) {
        parkTime.startSplit();

//        Canvas canvas = packet.fieldOverlay();

        // Radial vector towards the target:
        Vector2d radialVector = target.position.minus(drive.pose.position);
        double radialDistance = radialVector.norm();

        // We're done if the distance is small enough!
        // @@@ Check heading too
        if (radialDistance < 0.5)
            return false;

        double now = Actions.now();
        double deltaT = now - previousTime;

//        packet.put("deltaT", deltaT);

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

        parkPowerTime.startSplit();
        drive.setDrivePowers(null, velocity, acceleration);
        parkPowerTime.endSplit();

        // Remember stuff for the next iteration:
        previousVelocity = velocity;
        previousTime = now;

        // Draw the perpendicular vector in red, the radial vector in green:
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

        parkTime.endSplit();

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
        TimeSplitter startupTime = TimeSplitter.create("> Startup", false);
        TimeSplitter endTime = TimeSplitter.create("> End", false);
        TimeSplitter loopTime = TimeSplitter.create("> Loop");
        TimeSplitter poseTime = TimeSplitter.create("> Pose");
        TimeSplitter refineTime = TimeSplitter.create("> Refine");
        TimeSplitter driveTime = TimeSplitter.create("> Drive");
        TimeSplitter autoTime = TimeSplitter.create("> Auto");
        TimeSplitter ledTime = TimeSplitter.create("> Led");

        startupTime.startSplit();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Refiner refiner = new Refiner(hardwareMap);
        Led led = new Led(hardwareMap);
        startupTime.endSplit();
        AutoParker parker = null;

        // for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
        //     module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        // }

        waitForStart();

        while (opModeIsActive()) {

            // for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            //     module.clearBulkCache();
            // }

            loopTime.startSplit();

            // Update the telemetry pose and update the LED loop:
            poseTime.startSplit();
            drive.updatePoseEstimate();
            poseTime.endSplit();

            ledTime.startSplit();
            led.update();
            ledTime.endSplit();

            // Set up for visualizations:
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // Handle input:
            PoseVelocity2d manualPower = new PoseVelocity2d(new Vector2d(
                    stickShaper(-gamepad1.left_stick_y), stickShaper(-gamepad1.left_stick_x)),
                    stickShaper(-gamepad1.right_stick_x));

            boolean autoActivated = false;
            if (!gamepad1.a)
                parker = null;
            else {
                if (parker == null)
                    parker = new AutoParker(drive, new Pose2d(0, 0, 0));

                autoTime.startSplit();
                autoActivated = parker.park(packet);
                autoTime.endSplit();
            }

            if (!autoActivated) {
                driveTime.startSplit();
                drive.setDrivePowers(manualPower);
                driveTime.endSplit();
            }

            // Draw AprilTag poses and refine them:
            refineTime.startSplit();
            Pose2d refinedPose = refiner.refinePose(drive.pose, canvas);
            if (refinedPose != null) {
                led.setSteadyColor(Led.Color.GREEN);
                led.setPulseColor(Led.Color.RED, 0.25);
                drive.pose = refinedPose;
            }
            refineTime.endSplit();

            // Draw the best estimate pose:
            endTime.startSplit();
            canvas.setStroke("#3F51B5");
            MecanumDrive.drawRobot(canvas, drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            endTime.endSplit();

            loopTime.endSplit();
        }

        // Cleanup:
        refiner.close();
        TimeSplitter.logAllResults();
        led.setSteadyColor(Led.Color.OFF);
    }
}

