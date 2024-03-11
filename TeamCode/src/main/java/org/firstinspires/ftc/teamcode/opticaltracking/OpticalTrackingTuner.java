package org.firstinspires.ftc.teamcode.opticaltracking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Point;
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
    double tickDistance = 0; // @@@ Bug?

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
        double inchesPerTick;
        double correctionAngle; // Radians

        public Calibration(double inchesPerTick, double correctionAngle) {
            this.inchesPerTick = inchesPerTick;
            this.correctionAngle = correctionAngle;
        }
    }

    Calibration measureCalibration(OpticalTrackingPaa5100 optical, MecanumDrive drive) {
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
                    return new Calibration(0.001035, Math.toRadians(90.78));
                }
            }

            long xTotal = 0;
            long yTotal = 0;
            Calibration result = new Calibration(0, 0);
            optical.getMotion(); // Reset the sensor

            while (opModeIsActive() && !buttonA()) {
                // Query the push results:
                OpticalTrackingPaa5100.Motion motion = optical.getMotion();
                xTotal += motion.x;
                yTotal += motion.y;

                double tickDistance = Math.hypot(xTotal, yTotal);

                result.inchesPerTick = PUSH_INCHES / tickDistance;
                result.correctionAngle = -Math.atan2(yTotal, xTotal); // Rise over run

                telemetry.addLine(String.format("Accumulated ticks: (%d, %d), angle: %.2f°\n", xTotal, yTotal, Math.toDegrees(result.correctionAngle)));
                telemetry.addLine(String.format("Press A once you've pushed exactly %.1f inches.", PUSH_INCHES));
                telemetry.update();
            }

            telemetry.addLine(String.format("Test result: Inches per tick: %f,\n correction: %.3f°\n", result.inchesPerTick, Math.toDegrees(result.correctionAngle)));
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

    CenterOfRotation measureCenterOfRotation(OpticalTrackingPaa5100 optical, MecanumDrive drive, Calibration calibration) {
        while (opModeIsActive()) {
            telemetry.addLine(String.format("InchesPerTick: %f\n", calibration.inchesPerTick));
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

            Point currentPoint = new Point(0, 0);
            double rotationTotal = 0;
            double rotationTarget = REVOLUTION_COUNT * 2 * Math.PI;

            double farthestDistance = 0;
            Point farthestPoint = new Point(0, 0);
            tickDistance = 0;

            // Reset the two sensors:
            Globals.getImu().resetYaw();
            optical.getMotion();

            double lastYaw = Globals.getYaw();
            while (opModeIsActive() && (rotationTotal < rotationTarget)) {
                // Track the amount of rotation we've done:
                double yaw = Globals.getYaw();
                double deltaYaw = Globals.normalizeAngle(yaw - lastYaw);
                lastYaw = yaw;

                rotationTotal += Globals.normalizeAngle(deltaYaw);

                readTime.startSplit();
                OpticalTrackingPaa5100.Motion motion = optical.getMotion();
                readTime.endSplit();

                tickDistance += Math.hypot(motion.x, motion.y);

System.out.println(String.format("rotationTotal: %f, tickDistance: %f", rotationTotal, tickDistance));

                Point motionVector
                        = new Point(motion.x, motion.y).scale(calibration.inchesPerTick).rotate(calibration.correctionAngle);
                Point deltaPosition = deltaFieldPosition(yaw, motionVector.x, motionVector.y, deltaYaw);
                currentPoint = currentPoint.add(deltaPosition);
                points.add(currentPoint);

                double currentDistance = Math.hypot(currentPoint.x, currentPoint.y);
                if (currentDistance > farthestDistance) {
                    farthestDistance = currentDistance;
                    farthestPoint = currentPoint;
                }

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

            double inchesPerTick = calibration.inchesPerTick;
            double resultX = farthestPoint.x / 2;
            double resultY = farthestPoint.y / 2;
            double farthestPointRadius = farthestDistance / 2;
            double traveledRadius = tickDistance * inchesPerTick / (2 * REVOLUTION_COUNT * Math.PI);

            telemetry.addLine(String.format("InchesPerTick: %f, tickDistance: %f, farthestPoint.x: %f, farthestPoint.y: %f, traveledRadius: %f\n", inchesPerTick, tickDistance, farthestPoint.x, farthestPoint.y, tickDistance * inchesPerTick / (2 * REVOLUTION_COUNT * Math.PI)));
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

    @Override
    public void runOpMode() {
        Globals globals = new Globals(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), globals);
        TimeSplitter opticalInitialize = TimeSplitter.create("Optical Initialization");

        opticalInitialize.startSplit();
        OpticalTrackingPaa5100 optical = hardwareMap.get(OpticalTrackingPaa5100.class, "optical1");
        opticalInitialize.endSplit();

        waitForStart();

        Calibration calibration = measureCalibration(optical, drive);
        CenterOfRotation centerOfRotation = measureCenterOfRotation(optical, drive, calibration);

        telemetry.addLine("<< Completed all tuning tests! >>\n");
        telemetry.addLine(String.format("Inches per tick: %f", calibration.inchesPerTick));
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
