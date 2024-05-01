package org.firstinspires.ftc.teamcode.sparkfun;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Point;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.Stats;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="*OpticalTrackingTuner",group="Explore")
public class OpticalTrackingTuner extends LinearOpMode {
    final double REVOLUTION_COUNT = 1.0;
    final double RAMP_TIME = 0.5; // Seconds
    final double SPIN_SPEED = 0.2;
    final double PUSH_INCHES = 96; // 4 tiles

    boolean aPressed = false;
    boolean bPressed = false;
    double distanceTraveled = 0; // @@@ Bug?

    TimeSplitter readTime = TimeSplitter.create("getMotion");

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
        double distanceMultiplier;
        double correctionAngle; // Radians

        public Calibration(double distanceMultiplier, double correctionAngle) {
            this.distanceMultiplier = distanceMultiplier;
            this.correctionAngle = correctionAngle;
        }
    }

    @SuppressLint("DefaultLocale")
    Calibration measureCalibration(SparkFunOTOS optical, MecanumDrive drive) {
        while (opModeIsActive()) {
            drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            telemetry.addLine(String.format("Ready for calibration. Push the robot forward in a straight line for %.1f inches.\n", PUSH_INCHES));
            telemetry.addLine("Press A to start, B to skip");
            telemetry.update();
            while (opModeIsActive() && !buttonA()) {
                if (buttonB()) {
                    // Assume a reasonable default value:
                    return new Calibration(1.0, 0);
                }
            }

            Calibration result = new Calibration(1.0, 0);
            optical.resetTracking(); // Reset the sensor

            while (opModeIsActive() && !buttonA()) {
                // Query the push results:
                SparkFunOTOS.otos_pose2d_t pose = optical.getPosition();

                double measuredDistance = Math.hypot(pose.x, pose.y);
                result.correctionAngle = -Math.atan2(pose.y, pose.x); // Rise over run
                result.distanceMultiplier = (measuredDistance == 0) ? 0 : PUSH_INCHES / measuredDistance;

                telemetry.addLine(String.format("Measured pose: (%.1f, %.1f, %.1f°), distance: %.2f", pose.x, pose.y, pose.h, measuredDistance));
                telemetry.addLine(String.format("Computed angle: %.2f°\n", Math.toDegrees(result.correctionAngle)));
                telemetry.addLine(String.format("Press A once you've pushed exactly %.1f inches.", PUSH_INCHES));
                telemetry.update();
            }

            telemetry.addLine(String.format("Test result: Linear Scalar: %f,\nAngle Correction: %.3f°\n", result.distanceMultiplier, Math.toDegrees(result.correctionAngle)));
            telemetry.addLine("Press A to continue, B to repeat this test");
            telemetry.update();
            while (opModeIsActive() && !buttonB()) {
                if (buttonA())
                    return result; // ====>
            }
        }
        return null;
    }

    // Convert the robot-relative change-in-pose to field-relative change-in-position.
    // Apply the conversion via Forward Euler integration courtesy of Theorem 10.2.1
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf#page=194&zoom=100,57,447:
    static Point deltaFieldPosition(double theta, double deltaX, double deltaY, double deltaTheta) {
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);
        double sinDeltaThetaOverDeltaTheta = 1 - Math.pow(deltaTheta, 2) / 6;
        double cosDeltaThetaMinusOneOverDeltaTheta = -deltaTheta / 2;
        double oneMinusCosDeltaThetaOverDeltaTheta = deltaTheta / 2;
        double deltaXPrime
                = (cosTheta * sinDeltaThetaOverDeltaTheta - sinTheta * oneMinusCosDeltaThetaOverDeltaTheta) * deltaX
                + (cosTheta * cosDeltaThetaMinusOneOverDeltaTheta - sinTheta * sinDeltaThetaOverDeltaTheta) * deltaY;
        double deltaYPrime
                = (sinTheta * sinDeltaThetaOverDeltaTheta + cosTheta * oneMinusCosDeltaThetaOverDeltaTheta) * deltaX
                + (sinTheta * cosDeltaThetaMinusOneOverDeltaTheta + cosTheta * sinDeltaThetaOverDeltaTheta) * deltaY;
        return new Point(deltaXPrime, deltaYPrime);
    }

    static Point inferiorDeltaFieldPosition(double theta, double deltaX, double deltaY, double deltaTheta) {
        return new Point(deltaX, deltaY).rotate(theta + deltaTheta);
    }

    static class CenterOfRotation {
        double x;
        double y;
        double farthestPointRadius;
        double traveledRadius;
        double leastSquaresRadius;

        public CenterOfRotation(double x, double y, double farthestPointRadius, double traveledRadius, double leastSquaresRadius) {
            this.x = x;
            this.y = y;
            this.farthestPointRadius = farthestPointRadius;
            this.traveledRadius = traveledRadius;
            this.leastSquaresRadius = leastSquaresRadius;
        }
    }

    // Perform a least-squares fit of an array of points to a circle without using matrices,
    // courtesy of Copilot:
    static CenterOfRotation fitCircle(List<Point> points, double centerX, double centerY) {
        double radius = 0.0;

        // Iteratively refine the center and radius
        for (int iter = 0; iter < 100; iter++) {
            double sumX = 0.0;
            double sumY = 0.0;
            double sumR = 0.0;

            for (Point p : points) {
                double dx = p.x - centerX;
                double dy = p.y - centerY;
                double dist = Math.sqrt(dx * dx + dy * dy);
                sumX += dx / dist;
                sumY += dy / dist;
                sumR += dist;
            }

            centerX += sumX / points.size();
            centerY += sumY / points.size();
            radius = sumR / points.size();
        }

        return new CenterOfRotation(centerX, centerY,0, 0, radius);
    }

    @SuppressLint("DefaultLocale")
    CenterOfRotation measureCenterOfRotation(SparkFunOTOS optical, MecanumDrive drive, Calibration calibration) {
        while (opModeIsActive()) {
            telemetry.addLine(String.format("Distance multiplier: %f\n", calibration.distanceMultiplier));
            telemetry.addLine("Now ready for the center-of-rotation test.");
            telemetry.addLine("The robot will need room to spin.\n");
            telemetry.addLine("Press A to start, B to skip");
            telemetry.update();
            while (opModeIsActive() && !buttonA()) {
                if (buttonB())
                    return null;
            }

            telemetry.addLine(String.format("Rotating %.1f times...", REVOLUTION_COUNT));
            telemetry.update();

            ArrayList<Point> points = new ArrayList<>();
            rampMotors(drive, true);

            double rotationTotal = 0;
            double rotationTarget = REVOLUTION_COUNT * 2 * Math.PI;

            // Reset the two sensors:
            Globals.getImu().resetYaw();
            optical.resetTracking();

            double farthestDistance = 0;
            Point farthestPoint = new Point(0, 0);
            Point currentPoint = new Point(0, 0);
            SparkFunOTOS.otos_pose2d_t previousPose = optical.getPosition();
            distanceTraveled = 0;

            double lastYaw = Globals.getYaw();
            while (opModeIsActive() && (rotationTotal < rotationTarget)) {
                // Track the amount of rotation we've done:
                double yaw = Globals.getYaw();
                double deltaYaw = Globals.normalizeAngle(yaw - lastYaw);
                lastYaw = yaw;

                rotationTotal += Globals.normalizeAngle(deltaYaw);

                readTime.startSplit();
                SparkFunOTOS.otos_pose2d_t pose = optical.getPosition();
                readTime.endSplit();

                currentPoint = new Point(pose.x, pose.y);
                points.add(currentPoint);
                double distanceFromOrigin = Math.hypot(currentPoint.x, currentPoint.y);
                if (distanceFromOrigin > farthestDistance) {
                    farthestDistance = distanceFromOrigin;
                    farthestPoint = currentPoint;
                }

                distanceTraveled += Math.hypot(pose.x - previousPose.x, pose.y - previousPose.y) * calibration.distanceMultiplier;
                previousPose = pose;

                // Update the telemetry:
                double rotationsRemaining = (rotationTarget - rotationTotal) / (2 * Math.PI);
                telemetry.addLine(String.format("%.2f rotations remaining, %d points sampled", rotationsRemaining, points.size()));
                telemetry.update();

                // Draw the circle:
                TelemetryPacket packet = new TelemetryPacket();
                Canvas canvas = packet.fieldOverlay();
                double[] xPoints = new double[points.size()];
                double[] yPoints = new double[points.size()];
                for (int i = 0; i < points.size(); i++) {
                    xPoints[i] = points.get(i).x;
                    yPoints[i] = points.get(i).y;
                }
                canvas.setStroke("#00ff00");
                canvas.strokePolyline(xPoints, yPoints);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            rampMotors(drive, false);

            CenterOfRotation circleFit = fitCircle(points, farthestPoint.x / 2, farthestPoint.y / 2);

            double resultX = farthestPoint.x / 2;
            double resultY = farthestPoint.y / 2;
            double farthestPointRadius = farthestDistance / 2;
            double traveledRadius = distanceTraveled / (2 * REVOLUTION_COUNT * Math.PI);

            telemetry.addLine(String.format("Distance multiplier: %f, distance traveled: %f, farthestPoint.x: %f, farthestPoint.y: %f, traveledRadius: %f\n",
                    calibration.distanceMultiplier, distanceTraveled, farthestPoint.x, farthestPoint.y, traveledRadius));
            telemetry.addLine(String.format("Test result...\nOffset from center of rotation (inches): (%.2f, %.2f), samples: %d",
                    resultX, resultY, points.size()));
            telemetry.addLine(String.format("Farthest point radius: %.2f, Traveled radius: %.2f", farthestPointRadius, traveledRadius));
            telemetry.addLine(String.format("Error (inches): (%.2f, %.2f)\n", currentPoint.x, currentPoint.y));
            telemetry.addLine(String.format("Circle-fit position: (%.2f, %.2f), radius: %.2f\n", circleFit.x, circleFit.y, circleFit.leastSquaresRadius));

            telemetry.addLine("Press A to continue, B to repeat this test");
            telemetry.update();
            while (opModeIsActive() && !buttonB()) {
                if (buttonA()) {
                    return new CenterOfRotation(resultX, resultY, farthestPointRadius, traveledRadius, 0); // @@@
                }
            }
        }
        return null;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        TimeSplitter opticalInitialize = TimeSplitter.create("Optical Creation");
        TimeSplitter opticalCalibrate = TimeSplitter.create("Optical Calibration");

        Settings settings = new Settings(telemetry, gamepad1);
        Stats stats = new Stats();
        Globals globals = new Globals(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), globals);

        opticalInitialize.startSplit();
        SparkFunOTOS optical = hardwareMap.get(SparkFunOTOS.class, "sparkfun");
        opticalInitialize.endSplit();

        // Sensor position settings:
        optical.setOffset(new SparkFunOTOS.otos_pose2d_t(5.56, 3.39, 180));
        optical.setLinearScalar(0.956);
        optical.setAngularScalar(1.0);

        // IMU settings:
        opticalCalibrate.startSplit();
        optical.calibrateImu();
        opticalCalibrate.endSplit();

        optical.resetTracking();
        optical.setPosition(new SparkFunOTOS.otos_pose2d_t(0, 0, 0));

        waitForStart();

        Calibration calibration = measureCalibration(optical, drive);
        CenterOfRotation centerOfRotation = measureCenterOfRotation(optical, drive, calibration);

        telemetry.addLine("<< Completed all tuning tests! >>\n");
        telemetry.addLine(String.format("Distance multiplier: %f", calibration.distanceMultiplier));
        telemetry.addLine(String.format("Correction angle (degrees): %.2f", Math.toDegrees(calibration.correctionAngle)));
        if (centerOfRotation != null) {
            telemetry.addLine(String.format("Offset to center-of-rotation (inches): (%.3f, %.3f)", centerOfRotation.x, centerOfRotation.y));
            telemetry.addLine(String.format("\nFarthest-point to distance-traveled radius: %.2f", centerOfRotation.farthestPointRadius / centerOfRotation.traveledRadius));
        }
        telemetry.update();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive())
            ;

        TimeSplitter.reportAllResults();
    }
}
