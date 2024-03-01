package org.firstinspires.ftc.teamcode.opticaltracking;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;

@TeleOp(name="OpticalTrackingTuner",group="Explore")
public class OpticalTrackingTuner extends LinearOpMode {
    final int REVOLUTION_COUNT = 4;
    final double RAMP_TIME = 2; // Seconds
    final double SPIN_SPEED = 0.3;
    final double PUSH_INCHES = 96; // 4 tiles

    boolean aPressed = false;
    boolean bPressed = false;

    boolean buttonA() {
        if ((aPressed) && (!gamepad1.a)) {
            return true;
        }
        aPressed = gamepad1.a;
        return false;
    }

    boolean buttonB() {
        if ((bPressed) && (!gamepad1.b)) {
            return true;
        }
        bPressed = gamepad1.b;
        return false;
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

    Calibration measureCalibration(OpticalTrackingPaa5100 optical) {
        telemetry.addLine("Now ready for calibration.");
        telemetry.addLine(String.format("Push the robot forward in a straight line for %f inches.", PUSH_INCHES));
        telemetry.addLine("Press A to start");
        telemetry.update();
        while (!buttonA())
            ;

        Calibration result = new Calibration();
        long xTotal = 0;
        long yTotal = 0;

        while (!buttonA()) {
            // Query the push results:
            OpticalTrackingPaa5100.Motion motion = optical.getMotion();
            xTotal += motion.x;
            yTotal += motion.y;

            double tickDistance = Math.hypot(xTotal, yTotal);

            result.inchesPerTick = PUSH_INCHES / tickDistance;
            result.correctionAngle = Math.atan2(yTotal, xTotal); // Rise over run

            telemetry.addLine(String.format("Push forward %f inches.", PUSH_INCHES));
            telemetry.addLine(String.format("Total (x, y) ticks: %d, %d", xTotal, yTotal));
            telemetry.addLine("\nPress A when complete.");
            telemetry.update();
        }
        telemetry.addLine(String.format("Results: %f inches per tick, %.3f° correction\n", result.inchesPerTick, Math.toDegrees(result.correctionAngle)));
        return result;
    }

    double measureRadialDistance(OpticalTrackingPaa5100 optical, MecanumDrive drive, Calibration calibration) {
        telemetry.addLine("Now ready for the spin-distance test.");
        telemetry.addLine("The robot will need room to spin.\n");
        telemetry.addLine("Press A to start, B to skip");
        telemetry.update();
        while (!buttonA()) {
            if (buttonB())
                return -1;
        }

        telemetry.addLine(String.format("Rotating %d times...", REVOLUTION_COUNT));
        telemetry.update();

        // Ramp up the spin turning counter-clockwise:
        rampMotors(drive, true);

        optical.getMotion(); // Zero the motion
        double tickDistance = 0;
        double totalRotation = 0;
        double lastYaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        while (opModeIsActive() && (totalRotation < REVOLUTION_COUNT * 2 * Math.PI)) {
            // Track the amount of rotation we've done:
            double yaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            totalRotation += Globals.normalizeAngle(yaw - lastYaw);
            lastYaw = yaw;

            // Track the distance we've gone:
            OpticalTrackingPaa5100.Motion motion = optical.getMotion();
            tickDistance += Math.hypot(motion.x, motion.y);
        }

        rampMotors(drive, false);
        double spinDistance = tickDistance * calibration.inchesPerTick;
        telemetry.addLine(String.format("Radial distance: %f inches in radial distance.\n", spinDistance));
        return spinDistance;
    }

    static class CenterOfRotation {
        double x;
        double y;
        double radius;
    }

    // Least squares circle-fitting courtesy of Copilot:
    public static CenterOfRotation fitCircle(Point[] points) {
        double sumX = 0.0;
        double sumY = 0.0;
        double sumXX = 0.0;
        double sumYY = 0.0;
        double sumXY = 0.0;

        for (Point point : points) {
            double x = point.x;
            double y = point.y;
            sumX += x;
            sumY += y;
            sumXX += x * x;
            sumYY += y * y;
            sumXY += x * y;
        }

        double n = points.length;
        double a = 2 * (sumX * sumX + sumY * sumY);
        double b = -2 * (sumX * sumXX + sumY * sumXY);
        double c = sumXX * sumXX + sumXY * sumXY - n * sumYY * sumXX - n * sumXY * sumXY;

        double centerX = -b / (2 * a);
        double centerY = -c / (2 * a);
        double radius = Math.sqrt(centerX * centerX + centerY * centerY + c / a);

        CenterOfRotation result = new CenterOfRotation();
        result.x = centerX;
        result.y = centerY;
        result.radius = radius;

        return result;
    }

    CenterOfRotation measureCenterOfRotation(OpticalTrackingPaa5100 optical, MecanumDrive drive, Calibration calibration) {
        telemetry.addLine("Now ready for the center-of-rotation test.");
        telemetry.addLine("The robot will need room to spin.\n");
        telemetry.addLine("Press A to start, B to skip");
        telemetry.update();
        while (!buttonA()) {
            if (buttonB())
                return null;
        }

        telemetry.addLine(String.format("Rotating %d times...", REVOLUTION_COUNT));
        telemetry.update();

        ArrayList<Point> points = new ArrayList<>();
        rampMotors(drive, true);

        Point currentPoint = new Point(0, 0);
        double totalRotation = 0;
        double lastYaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        while (opModeIsActive() && (totalRotation < REVOLUTION_COUNT * 2 * Math.PI)) {
            // Track the amount of rotation we've done:
            double yaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            totalRotation += Globals.normalizeAngle(yaw - lastYaw);
            lastYaw = yaw;

            OpticalTrackingPaa5100.Motion motion = optical.getMotion();
            currentPoint.add(new Point(motion.x, motion.y).rotate(calibration.correctionAngle).scale(calibration.inchesPerTick));
            points.add(currentPoint);
        }
        Point[] array = new Point[points.size()];
        CenterOfRotation result = fitCircle(points.toArray(array));

        telemetry.addLine(String.format("Center of rotation (inches): (%.2f, %.2f)  Samples: %d\n", result.x, result.y, points.size()));
        return result;
    }

    @Override
    public void runOpMode() {
        Globals.initialize(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        OpticalTrackingPaa5100 optical = hardwareMap.get(OpticalTrackingPaa5100.class, "optical2");

        waitForStart();

        Calibration calibration = measureCalibration(optical);
        CenterOfRotation centerOfRotation = measureCenterOfRotation(optical, drive, calibration);
        double radialDistance = measureRadialDistance(optical, drive, calibration);

        telemetry.addLine("<< Completed all tests! >>\n\n");
        telemetry.addLine(String.format("    Inches per tick: %f", calibration.inchesPerTick));
        telemetry.addLine(String.format("    Correction angle (degrees): %.2f", Math.toDegrees(calibration.correctionAngle)));
        if (centerOfRotation != null) {
            telemetry.addLine(String.format("    Offset to center-of-rotation (inches): (%.2f, %.2f)", centerOfRotation.x, centerOfRotation.y));
        }
        if (radialDistance >= 0) {
            telemetry.addLine(String.format("    Radial distance (inches): %.2f", radialDistance));
        }
        if ((centerOfRotation != null) && (radialDistance >= 0)) {
            telemetry.addLine(String.format("    Center-of-rotation to radial distance: %.2f", radialDistance / centerOfRotation.radius));
        }
        telemetry.addLine("\nPress STOP once you've recorded everything.");
        telemetry.update();
        while (opModeIsActive())
            ;
    }
}
