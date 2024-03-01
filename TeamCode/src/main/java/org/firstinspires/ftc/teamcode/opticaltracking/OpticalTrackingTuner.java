package org.firstinspires.ftc.teamcode.opticaltracking;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;

@TeleOp(name="*OpticalTrackingTuner",group="Explore")
public class OpticalTrackingTuner extends LinearOpMode {
    final int REVOLUTION_COUNT = 1;
    final double RAMP_TIME = 2; // Seconds
    final double SPIN_SPEED = 0.3;
    final double PUSH_INCHES = 24; // @@@ 96; // 4 tiles

    boolean aPressed = false;
    boolean bPressed = false;

    boolean buttonA() {
        boolean result = (aPressed) && (!gamepad1.a);
        aPressed = gamepad1.a;
        return result;
    }

    boolean buttonB() {
        boolean result = (bPressed) && (!gamepad1.b);
        bPressed = gamepad1.b;
        return result;
    }

    void rampMotors(MecanumDrive drive, boolean up) {
        double startTime = Globals.time();
        while (opModeIsActive()) {
            double duration = Globals.time() - startTime;
            double fraction = (up) ? (duration / RAMP_TIME) : (1 - duration / RAMP_TIME);
            drive.rightFront.setPower(fraction * SPIN_SPEED);
            drive.rightBack.setPower(fraction * SPIN_SPEED);
            drive.leftFront.setPower(-fraction * SPIN_SPEED);
            drive.leftBack.setPower(-fraction * SPIN_SPEED);
            if (duration > RAMP_TIME)
                break; // ===>
        }
    }

    static class Calibration {
        double inchesPerTick;
        double correctionAngle; // Radians
    }

    Calibration measureCalibration(OpticalTrackingPaa5100 optical, MecanumDrive drive) {
        Calibration result = new Calibration();
        while (true) {
            telemetry.addLine(String.format("Ready for calibration. Push the robot forward in a straight line for %.1f inches.\n", PUSH_INCHES));
            telemetry.addLine("Press A to start");
            telemetry.update();
            while (opModeIsActive() && !buttonA())
                ;

            drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            long xTotal = 0;
            long yTotal = 0;

            optical.getMotion(); // Reset the sensor
            while (opModeIsActive() && !buttonA()) {
                // Query the push results:
                OpticalTrackingPaa5100.Motion motion = optical.getMotion();
                xTotal += motion.x;
                yTotal += motion.y;

                double tickDistance = Math.hypot(xTotal, yTotal);

                result.inchesPerTick = PUSH_INCHES / tickDistance;
                result.correctionAngle = Math.atan2(yTotal, xTotal); // Rise over run

                telemetry.addLine(String.format("Accumulated ticks: (%d, %d), angle: %.2f°", xTotal, yTotal, Math.toDegrees(result.correctionAngle)));
                telemetry.addLine(String.format("Press A once you've pushed %.1f inches.", PUSH_INCHES));
                telemetry.update();
            }
            telemetry.addLine(String.format("Test result: Inches per tick: %f, correction: %.3f°\n", result.inchesPerTick, Math.toDegrees(result.correctionAngle)));
            telemetry.addLine("Press A to continue, B to repeat this test");
            while (opModeIsActive() && !buttonB()) {
                if (buttonA())
                    return result; // ====>
            }
        }
    }

    double measureRadialDistance(OpticalTrackingPaa5100 optical, MecanumDrive drive, Calibration calibration) {
        telemetry.addLine("Now ready for the spin-distance test.");
        telemetry.addLine("The robot will need room to spin.\n");
        telemetry.addLine("Press A to start, B to skip");
        telemetry.update();
        while (opModeIsActive() && !buttonA()) {
            if (buttonB())
                return -1;
        }

        telemetry.addLine(String.format("Rotating %d times...", REVOLUTION_COUNT));
        telemetry.update();

        // Ramp up the spin turning counter-clockwise:
        rampMotors(drive, true);

        optical.getMotion(); // Zero the motion
        double tickDistance = 0;
        double rotationTotal = 0;
        double rotationTarget = REVOLUTION_COUNT * 2 * Math.PI;

        // Reset the two sensors:
        drive.imu.resetYaw();
        optical.getMotion();
        double lastYaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        while (opModeIsActive() && (rotationTotal < rotationTarget)) {
            // Track the amount of rotation we've done:
            double yaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            rotationTotal += Globals.normalizeAngle(yaw - lastYaw);
            lastYaw = yaw;

            // Track the distance we've gone:
            OpticalTrackingPaa5100.Motion motion = optical.getMotion();
            tickDistance += Math.hypot(motion.x, motion.y);

            double rotationsRemaining = (rotationTarget - rotationTotal) / (2 * Math.PI);
            telemetry.addLine(String.format("%.2f rotations remaining, %.0f ticks traveled", rotationsRemaining, tickDistance));
            telemetry.update();
        }

        rampMotors(drive, false);

        // Circumference = 2 * radius * pi
        double radius = tickDistance * calibration.inchesPerTick / (2 * REVOLUTION_COUNT * Math.PI);
        telemetry.addLine(String.format("Test result: radial distance: %.2f inches\n", radius));
        return radius;
    }

    static class CenterOfRotation {
        double x;
        double y;
        double radius;
    }

    CenterOfRotation measureCenterOfRotation(OpticalTrackingPaa5100 optical, MecanumDrive drive, Calibration calibration) {
        telemetry.addLine("Now ready for the center-of-rotation test.");
        telemetry.addLine("The robot will need room to spin.\n");
        telemetry.addLine("Press A to start, B to skip");
        telemetry.update();
        while (opModeIsActive() && !buttonA()) {
            if (buttonB())
                return null;
        }

        telemetry.addLine(String.format("Rotating %d times...", REVOLUTION_COUNT));
        telemetry.update();

        ArrayList<Point> points = new ArrayList<>();
        rampMotors(drive, true);

        Point currentPoint = new Point(0, 0);
        double rotationTotal = 0;
        double rotationTarget = REVOLUTION_COUNT * 2 * Math.PI;

        double farthestDistance = 0;
        Point farthestPoint = new Point(0, 0);
        double tickDistance = 0;

        // Reset the two sensors:
        drive.imu.resetYaw();
        optical.getMotion();

        double lastYaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        while (opModeIsActive() && (rotationTotal < rotationTarget)) {
            // Track the amount of rotation we've done:
            double yaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            rotationTotal += Globals.normalizeAngle(yaw - lastYaw);
            lastYaw = yaw;

            OpticalTrackingPaa5100.Motion motion = optical.getMotion();
            tickDistance += Math.hypot(motion.x, motion.y);

            currentPoint = currentPoint.add(new Point(motion.x, motion.y).rotate(calibration.correctionAngle + yaw).scale(calibration.inchesPerTick));
            points.add(currentPoint);

            double currentDistance = Math.hypot(currentPoint.x, currentPoint.y);
            if (currentDistance > farthestDistance) {
                farthestDistance = currentDistance;
                farthestPoint = currentPoint;
            }

            double rotationsRemaining = (rotationTarget - rotationTotal) / (2 * Math.PI);
            telemetry.addLine(String.format("%.2f rotations remaining, %d points sampled", rotationsRemaining, points.size()));
            telemetry.update();
        }

        rampMotors(drive, false);

        double traveledRadius = tickDistance * calibration.inchesPerTick / (2 * REVOLUTION_COUNT * Math.PI);

        CenterOfRotation result = new CenterOfRotation();
        result.x = -farthestPoint.x / 2;
        result.y = -farthestPoint.y / 2;
        result.radius = farthestDistance / 2;

        telemetry.addLine(String.format("Test result: Offset from center of rotation (inches): (%.2f, %.2f), samples: %d",
                result.x, result.y, points.size()));
        telemetry.addLine(String.format("Farthest point radius: %.2f, Traveled radius: %.2f", result.radius, traveledRadius));
        telemetry.addLine(String.format("End point (inches): (%.2f, %.2f)\n", currentPoint.x, currentPoint.y));
        return result;
    }

    @Override
    public void runOpMode() {
        Globals.initialize(telemetry);
        OpticalTrackingPaa5100 optical = hardwareMap.get(OpticalTrackingPaa5100.class, "optical2");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        Calibration calibration = measureCalibration(optical, drive);
        CenterOfRotation centerOfRotation = measureCenterOfRotation(optical, drive, calibration);
        double radialDistance = measureRadialDistance(optical, drive, calibration);

        telemetry.addLine("<< Completed all tuning tests! >>\n");
        telemetry.addLine(String.format("Inches per tick: %f", calibration.inchesPerTick));
        telemetry.addLine(String.format("Correction angle (degrees): %.2f", Math.toDegrees(calibration.correctionAngle)));
        if (centerOfRotation != null) {
            telemetry.addLine(String.format("Offset to center-of-rotation (inches): (%.2f, %.2f)", centerOfRotation.x, centerOfRotation.y));
        }
        if (radialDistance >= 0) {
            telemetry.addLine(String.format("Radial distance (inches): %.2f", radialDistance));
        }
        if ((centerOfRotation != null) && (radialDistance >= 0)) {
            telemetry.addLine(String.format("Center-of-rotation to radial distance ratio: %.2f", radialDistance / centerOfRotation.radius));
        }
        telemetry.update();
        while (opModeIsActive())
            ;
    }
}
