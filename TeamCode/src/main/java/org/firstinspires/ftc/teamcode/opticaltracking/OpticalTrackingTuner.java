package org.firstinspires.ftc.teamcode.opticaltracking;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="OpticalTrackingTuner",group="Explore")
public class OpticalTrackingTuner extends LinearOpMode {
    final int REVOLUTION_COUNT = 4;
    final double RAMP_TIME = 2; // Seconds
    final double SPIN_SPEED = 0.3;
    final double PUSH_INCHES = 96; // 4 tiles

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.initialize(telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        OpticalTrackingPaa5100 optical = hardwareMap.get(OpticalTrackingPaa5100.class, "optical2");
        double spinDistance = 0;
        long xTotal = 0;
        long yTotal = 0;
        double inchesPerTick = 0;
        double angle = 0;

        waitForStart();

        while (opModeIsActive()) {
            Globals.startLoop();

            if (gamepad1.a) {
                // Ramp up the spin turning counter-clockwise:
                double startTime = Globals.time();
                while (opModeIsActive()) {
                    double duration = Globals.time() - startTime;
                    double fraction = duration / RAMP_TIME;
                    drive.rightFront.setPower(fraction * SPIN_SPEED);
                    drive.rightBack.setPower(fraction * SPIN_SPEED);
                    drive.leftFront.setPower(-fraction * SPIN_SPEED);
                    drive.leftBack.setPower(-fraction * SPIN_SPEED);
                    if (duration > RAMP_TIME)
                        break; // ===>
                }

                double lastYaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                optical.getMotion(); // Zero the motion
                double totalRotation = 0;
                spinDistance = 0;
                while (opModeIsActive() && (totalRotation < REVOLUTION_COUNT * 2 * Math.PI)) {
                    // Track the amount of rotation we've done:
                    double yaw = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    totalRotation += Globals.normalizeAngle(yaw - lastYaw);
                    lastYaw = yaw;

                    // Track the distance we've gone:
                    OpticalTrackingPaa5100.Motion motion = optical.getMotion();
                    spinDistance += Math.hypot(motion.x, motion.y);
                }

                // Ramp down the spin:
                startTime = Globals.time();
                while (opModeIsActive()) {
                    double duration = Globals.time() - startTime;
                    double fraction = 1 - duration / RAMP_TIME;
                    drive.rightFront.setPower(fraction * SPIN_SPEED);
                    drive.rightBack.setPower(fraction * SPIN_SPEED);
                    drive.leftFront.setPower(-fraction * SPIN_SPEED);
                    drive.leftBack.setPower(-fraction * SPIN_SPEED);
                    if (duration > RAMP_TIME)
                        break; // ===>
                }
            }
            if (spinDistance == 0) {
                // Query the push results:
                OpticalTrackingPaa5100.Motion motion = optical.getMotion();
                xTotal += motion.x;
                yTotal += motion.y;

                double tickDistance = Math.hypot(xTotal, yTotal);
                inchesPerTick = PUSH_INCHES / tickDistance;
                angle = Math.atan2(yTotal, xTotal); // Rise over run

                Globals.telemetry.addLine("Push forward %d inches and then record the results.\n");
                Globals.telemetry.addLine("Press A to begin the spin test.\n\n");

                Globals.telemetry.addLine(String.format("Distance total (ticks): %d, %d\n", xTotal, yTotal));
                Globals.telemetry.addLine(String.format("Assuming %.1f inches...\n", PUSH_INCHES));
            }
            Globals.telemetry.addLine(String.format("    Inches-per-tick: %.2f\n", inchesPerTick));
            Globals.telemetry.addLine(String.format("    Sensor angle: %.2f°\n", Math.toDegrees(angle)));
            if (spinDistance != 0) {
                Globals.telemetry.addLine(String.format("Spin total (ticks): %.1f", spinDistance));
                Globals.telemetry.addLine(String.format("Spin total (inches assuming InchesPerTick): %.1f", spinDistance * inchesPerTick));
            }
            Globals.endLoop();
        }

        TimeSplitter.reportAllResults();
    }
}
