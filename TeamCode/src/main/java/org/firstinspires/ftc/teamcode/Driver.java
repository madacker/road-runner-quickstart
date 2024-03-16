package org.firstinspires.ftc.teamcode;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * Led control class.
 * @noinspection FieldCanBeLocal
 */
class Led {
    public enum Color { OFF, RED, GREEN }

    private Color pulseColor = Color.OFF;
    private Color steadyColor = Color.OFF;
    private double pulseEndTime = -1.0; // Seconds

    private DigitalChannel redLed;
    private DigitalChannel greenLed;

    public Led(HardwareMap hardwareMap) {
        redLed = hardwareMap.get(DigitalChannel.class, "red");
        greenLed = hardwareMap.get(DigitalChannel.class, "green");

        // The mode defaults to 'input':
        redLed.setMode(DigitalChannel.Mode.OUTPUT);
        greenLed.setMode(DigitalChannel.Mode.OUTPUT);

        setSteadyColor(Color.OFF);
    }

    private void setColor(Color color) {
        boolean redOn = false;
        boolean greenOn = false;
        if (color == Color.RED) {
            redOn = true;
        } else if (color == Color.GREEN) {
            greenOn = true;
        }
        redLed.setState(!redOn);
        greenLed.setState(!greenOn);
    }

    /** @noinspection BooleanMethodIsAlwaysInverted*/
    private boolean isPulseActive() {
        return ((pulseEndTime != -1.0) && (nanoTime() * 1e-9 < pulseEndTime));
    }

    public void setSteadyColor(Color color) {
        steadyColor = color;
        if (!isPulseActive())
            setColor(color);
    }

    public void setPulseColor(Color color, double  seconds) {
        pulseColor = color;
        pulseEndTime = nanoTime() * 1e-9 + seconds; // Seconds
        setColor(color);
    }

    public void update() {
        if ((pulseEndTime != -1.0) && (!isPulseActive())) {
            pulseEndTime = -1.0;
            setColor(steadyColor);
        }
    }
}

@Config
class AutoParker {
    Poser poser;                    // Get the pose and pose velocity
    MecanumDrive drive;             // Used to set the motors
    Pose2d target;                  // Target pose
    double facingOrientation;       // Orientation of robot to point to goal when far from goal
    Vector2d originalRadialVector;  // Original radial vector
    double turningDistance;         // Distance at which to start turning to final orientation
    double previousTime;            // Previous time in seconds
    Vector2d originalTangentVector; // Normalized version of original tangent vector
    double radialSpeed;             // Inches/s, can be negative
    double tangentSpeed;            // Inches/s, can be negative
    double angularSpeed;            // Radians/s, can be negative
    boolean isPidEngaged;           // True if close to the target and the PID is engaged
    double previousRadialLength;    // Previous radial length to feed into the PID
    public static double kp = 0;    // PID proportional constant
    public static double kd = 0;    // PID derivative constant

    double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2*Math.PI;
        while (angle < -Math.PI)
            angle += 2*Math.PI;
        return angle;
    }

    AutoParker(Poser poser, MecanumDrive drive, Pose2d target, double facingOrientation, double turningDistance) {
        this.poser = poser;
        this.drive = drive;
        this.target = target;
        this.facingOrientation = facingOrientation;
        this.turningDistance = turningDistance;

        // Remember the time:
        this.previousTime = Actions.now();

        // Position --------------------------------------------------------------------------------

        // Radial vector towards the target:
        originalRadialVector = target.position.minus(poser.bestPose.position);

        // Tangent vector -90 degrees from radial:
        // noinspection SuspiciousNameCombination
        originalTangentVector = new Vector2d(originalRadialVector.y, -originalRadialVector.x);

        // Normalize the tangent, being careful to protect against divide-by-zero:
        double tangentLength = originalTangentVector.norm();
        if (tangentLength != 0)
            originalTangentVector = originalTangentVector.div(tangentLength);

        // Convert the velocity to field-relative coordinates:
        PoseVelocity2d velocity = poser.bestPose.times(poser.velocity);

        // Compute the current speed:
        double speed = Math.hypot(velocity.linearVel.x, velocity.linearVel.y);

        // Compute the angle from the robot's current direction to the target:
        double theta = Math.atan2(originalRadialVector.y, originalRadialVector.x)
                     - Math.atan2(velocity.linearVel.y, velocity.linearVel.x);

        // Compute the component speeds:
        tangentSpeed = Math.sin(theta) * speed;
        radialSpeed = Math.cos(theta) * speed;

        // Handle heading:
        angularSpeed = velocity.angVel;
    }

    /**
     * Returns false when done, true while still working on it:
     * @noinspection CommentedOutCode
      */
    boolean park() {
        // Radial vector towards the target:
        Vector2d radialVector = target.position.minus(poser.bestPose.position);
        double radialLength = radialVector.norm();

        // Angular distance to the target:
        double angularDelta = normalizeAngle(target.heading.log() - poser.bestPose.heading.log());

        // We're done if the distance is small enough to the goal!
//  @@@       if ((radialLength < 0.5) && (Math.abs(angularDelta) < Math.toRadians(2)))
//            return false;

        // When far away, point the robot to the goal instead of beginning to turn to its final
        // orientation:
        if (radialLength > turningDistance) {
            double headingToGoal = Math.atan2(radialVector.y, radialVector.x) + facingOrientation;
            angularDelta = normalizeAngle(headingToGoal - poser.bestPose.heading.log());
        }

        double now = Actions.now();
        double deltaT = now - previousTime;

        // Position --------------------------------------------------------------------------------

        // Decrease the magnitude of the tangent speed with a minimum value of zero:
        double tangentMagnitude = Math.max(0,
                Math.abs(tangentSpeed) + MecanumDrive.PARAMS.minProfileAccel * deltaT);

        // Convert to the signed tangent speed:
        tangentSpeed = Math.signum(tangentSpeed) * tangentMagnitude;

        // If we've passed the target, engage the PID:
        if (originalRadialVector.dot(radialVector) < 0)
            isPidEngaged = true;

        if (isPidEngaged) {
            // Use the PD controller:
            radialSpeed = kp * radialLength + kd * (radialLength - previousRadialLength);
        } else {
            // Increase the speed towards the target:
            double increasingRadialSpeed = radialSpeed + MecanumDrive.PARAMS.maxProfileAccel * deltaT;

            // Compute the maximum possible speed towards the target, accounting for the speed along
            // the tangent:
            double maxRadialSpeed = Math.hypot(tangentSpeed, MecanumDrive.PARAMS.maxWheelVel);

            // Calculate the maximum speed directly towards the target assuming constant
            // deceleration. We use the kinematic equation "v^2 = u^2 + 2as" where v is the
            // current velocity, u is the initial velocity, a is the acceleration, s is distance
            // traveled. We apply it in reverse:
            double radialApproach = Math.sqrt(2 * Math.abs(MecanumDrive.PARAMS.minProfileAccel) * radialLength);

            // Set the new radial speed as the minimum of the three:
            radialSpeed = Math.min(Math.min(increasingRadialSpeed, maxRadialSpeed), radialApproach);
        }

        Vector2d radialVelocity = radialVector.div(radialLength).times(radialSpeed);
        Vector2d tangentVelocity = originalTangentVector.times(tangentSpeed);

        // Heading ---------------------------------------------------------------------------------

        // Calculate the angular velocity as a positive magnitude:
        if (angularDelta < 0) {
            double increasingAngularSpeed = angularSpeed - MecanumDrive.PARAMS.maxAngAccel * deltaT;
            double maxAngularSpeed = -MecanumDrive.PARAMS.maxAngVel;
            double angularApproach = -Math.sqrt(2 * MecanumDrive.PARAMS.maxAngAccel * Math.abs(angularDelta));
            angularSpeed = Math.max(Math.max(increasingAngularSpeed, maxAngularSpeed), angularApproach);
        } else {
            double increasingAngularSpeed = angularSpeed + MecanumDrive.PARAMS.maxAngAccel * deltaT;
            double maxAngularSpeed = MecanumDrive.PARAMS.maxAngVel;
            double angularApproach = Math.sqrt(2 * MecanumDrive.PARAMS.maxAngAccel * Math.abs(angularDelta));
            angularSpeed = Math.min(Math.min(increasingAngularSpeed, maxAngularSpeed), angularApproach);
        }

        // Combined --------------------------------------------------------------------------------

        // Add the component velocities to get our new velocity:
        PoseVelocity2d velocity = new PoseVelocity2d(radialVelocity.plus(tangentVelocity), angularSpeed);
        drive.setDrivePowers(poser.bestPose, poser.velocity, null, velocity);

        // Remember stuff for the next iteration:
        previousTime = now;
        previousRadialLength = radialLength;

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
    Poser poser;
    MecanumDrive drive;
    Vector2d wallPoint;
    Vector2d wallVector;

    // The wall will repulse any robot approaching from the left of the vector.
    Wall(Poser poser, MecanumDrive drive, Vector2d point, Vector2d vector) {
        this.poser = poser; this.drive = drive; wallPoint = point; wallVector = vector;
    }

    /**
     * The velocity must be field-relative, calibrated inches/s and radians/s:
     * @noinspection UnusedReturnValue
     */
    PoseVelocity2d repulse(PoseVelocity2d velocity) {

        // Length of the velocity vector:
        double velocityMagnitude = Math.hypot(velocity.linearVel.x, velocity.linearVel.y);
        if (velocityMagnitude == 0)
            return velocity;

        // Direction that's normal to the wall, pointing towards the wall from the robot's
        // perspective:
        double normalAngle = Math.atan2(wallVector.y, wallVector.x) - Math.PI/2; // Rise over run

        // Direction of the velocity vector:
        double velocityAngle = Math.atan2(velocity.linearVel.y, velocity.linearVel.x);

        // Break the velocity vector into constituent velocities towards and along the wall:
        double towardWallAngle = velocityAngle - normalAngle;
        double towardWallVelocity = Math.cos(towardWallAngle) * velocityMagnitude;
        double alongWallVelocity = Math.sin(towardWallAngle) * velocityMagnitude;

        // Compute the distance to the wall:
        double dx = wallPoint.x - poser.bestPose.position.x;
        double dy = wallPoint.y - poser.bestPose.position.y;
        double pointToPointDistance = Math.hypot(dy, dx);
        double pointToPointAngle = Math.atan2(dy, dx);
        double distance = Math.cos(pointToPointAngle - towardWallAngle) * pointToPointDistance;

        // Calculate the maximum speed directly towards the target assuming constant
        // deceleration. We use the kinematic equation "v^2 = u^2 + 2as" where v is the
        // current velocity, u is the initial velocity, a is the acceleration, s is distance
        // traveled. We apply it in reverse:
        double maxApproachVelocity = Math.sqrt(2 * Math.abs(MecanumDrive.PARAMS.minProfileAccel) * distance);
        towardWallVelocity = Math.min(towardWallVelocity, maxApproachVelocity);

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

// @Photon
@TeleOp(name="Driver", group="Aardvark")
public class Driver extends LinearOpMode {
    // Shape the stick input for more precision at slow speeds:
    public double shapeStick(double stickValue) {
        double result = Math.signum(stickValue) * Math.abs(Math.pow(stickValue, 2.0));
        result = Math.min(result, 1.0);
        result = Math.max(result, -1.0);
        return result;
    }

    // Scale the stick input to a shaped result that accounts for the dead zone:
    public double scaleStick(double inputValue, double scale) {
//        final double DEAD_ZONE = 0.10; // Assume 15%, not forgetting that the range is [-1, 1]
//        if (Math.abs(inputValue) <= DEAD_ZONE) {
//            Globals.telemetry.addLine(String.format("Raw stick: %.2f, Result: --", inputValue));
//            return 0;
//        }

        // Shape the result:
        double shapedValue = shapeStick(inputValue);

//        // Push away from the dead zone:
//        shapedValue = (1.0 - DEAD_ZONE) * shapedValue + Math.signum(shapedValue) * DEAD_ZONE;

//        Globals.telemetry.addLine(String.format("Raw stick: %.2f, Result: %.2f, Scale to: %.2f",
//                inputValue, shapedValue, scale));

        // Return the scaled result:
        return shapedValue * scale;
    }

    /** @noinspection ConstantValue*/
    @Override
    public void runOpMode() throws InterruptedException {
        Settings settings = new Settings(telemetry, gamepad1);
        Stats stats = new Stats();
        Globals globals = new Globals(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), globals);

        Poser poser = new Poser(hardwareMap, drive, null);
        Led led = new Led(hardwareMap);
        Wall wall = new Wall(poser, drive, new Vector2d(-72, -36), new Vector2d(24, -24));
        AutoParker parker = null;

        // Feed forward model: voltage = kS + kV*velocityInTicksPerSecond + kA*acceleration
        double fullAxialSpeed
                = ((Globals.getVoltage() - MecanumDrive.PARAMS.kS) / MecanumDrive.PARAMS.kV)
                * MecanumDrive.PARAMS.inPerTick;
        double fullLateralSpeed
                = fullAxialSpeed * MecanumDrive.PARAMS.lateralInPerTick / MecanumDrive.PARAMS.inPerTick;
        // Rotations/s = fullAxialSpeed / (pi * wheelbase)
        // Radians/s = 2 * pi * rotations/s
        double fullAngularSpeed
                = 2 * fullAxialSpeed / (MecanumDrive.PARAMS.trackWidthTicks * MecanumDrive.PARAMS.inPerTick);

        // TODO: Should integrate theoretical here:
        Settings.registerListOption("Max speed factor", new String[] {"0.25", "0.50", "0.75", "1.0"}, 1,
                (i, string) -> { double factor = Double.parseDouble(string); MecanumDrive.PARAMS.maxWheelVel = 60 * factor; MecanumDrive.PARAMS.maxAngVel = Math.PI * factor; });

        waitForStart();

        while (opModeIsActive()) {
            Globals.startLoop();
            Gamepad gamepad1 = settings.update();
            stats.update();

            Stats.startTimer("* totalPose");
            poser.update();
            Stats.endTimer("* totalPose");

            led.update();

            if (poser.isLockedOn())
                led.setSteadyColor(Led.Color.RED);

            // The 'A' button activates the auto-parker:
            boolean parkingActivated = false;

            // We can't use input when the settings menu is up:
            if ((gamepad1 == null) || (!gamepad1.a))
                parker = null;
            else {
                if (poser.isLockedOn()) {
                    if (parker == null)
                        parker = new AutoParker(poser, drive, new Pose2d(45, 36, Math.PI),
                                Math.PI, 48);
                    parkingActivated = parker.park();
                }
            }

            // The 'left-bumper' button activates Road Runner homing:
            boolean roadrunnerActivated = drive.doActionsWork(poser.bestPose, poser.velocity, Globals.packet);
            if ((gamepad1 != null) && (gamepad1.left_bumper) && (!roadrunnerActivated) && (poser.isLockedOn())) {
                // Ensure that velocity is zero-ish:
                if ((Math.abs(poser.velocity.linearVel.x) < 0.1) &&
                        (Math.abs(poser.velocity.linearVel.y) < 0.1) &&
                        (Math.abs(poser.velocity.angVel) < 0.1)) {

                    boolean toBackdrop = poser.bestPose.position.x < 6.0;
                    Action action;
                    if (toBackdrop) {
                        // Go to the blue backdrop:
                        Pose2d keyPose = new Pose2d(-36, -36, Math.PI);
                        double startTangent = Math.atan2(
                                keyPose.position.y - poser.bestPose.position.y,
                                keyPose.position.x - poser.bestPose.position.x);
                        action = drive.actionBuilder(poser.bestPose)
                                .setTangent(startTangent)
                                .splineToSplineHeading(keyPose, 0)
                                .splineTo(new Vector2d(12, -36), 0)
                                .splineTo(new Vector2d(48, 36), 0)
                                .build();
                    } else {
                        // Go to the blue wing:
                        Pose2d keyPose = new Pose2d(12, -36, Math.PI);
                        double startTangent = Math.atan2(
                                keyPose.position.y - poser.bestPose.position.y,
                                keyPose.position.x - poser.bestPose.position.x);
                        action = drive.actionBuilder(poser.bestPose)
                                .setTangent(startTangent)
                                .splineToSplineHeading(keyPose, Math.PI)
                                .splineTo(new Vector2d(-36, -36), Math.PI)
                                .splineTo(new Vector2d(-72 + 24, -72 + 24), Math.toRadians(225))
                                .build();
                    }
                    drive.runParallel(action);
                    roadrunnerActivated = true;
                }
            }

            // Manually drive:
            if ((!parkingActivated) && (!roadrunnerActivated)) {
                //noinspection ConstantValue
                if (true) {
                    PoseVelocity2d calibratedVelocity = new PoseVelocity2d(new Vector2d(
                            scaleStick(-this.gamepad1.left_stick_y, MecanumDrive.PARAMS.maxWheelVel), // fullAxialSpeed),
                            scaleStick(-this.gamepad1.left_stick_x, MecanumDrive.PARAMS.maxWheelVel)), // fullLateralSpeed)),
                            scaleStick(-this.gamepad1.right_stick_x, MecanumDrive.PARAMS.maxAngVel)); // , fullAngularSpeed));

                    PoseVelocity2d fieldVelocity = poser.bestPose.times(calibratedVelocity);

                    if ((wall != null) && (poser.isLockedOn()))
                        wall.repulse(fieldVelocity); // @@@

                    drive.setDrivePowers(poser.bestPose, poser.velocity, null, fieldVelocity);
                } else {
                    PoseVelocity2d manualPower = new PoseVelocity2d(new Vector2d(
                            shapeStick(-this.gamepad1.left_stick_y),
                            shapeStick(-this.gamepad1.left_stick_x)),
                            shapeStick(-this.gamepad1.right_stick_x));
                    drive.setDrivePowers(manualPower);
                }
            }

            // Finish up:
            Stats.updateVelocity(poser.velocity, fullAxialSpeed);
            Globals.endLoop();
        }

        // Cleanup:
        if (poser != null) {
            poser.close();
        }
        if (led != null) {
            led.setSteadyColor(Led.Color.OFF);
        }

        // Output summary:
        TimeSplitter.reportAllResults();
    }
}

