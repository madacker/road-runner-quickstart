/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.explorations;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@TeleOp(name="AprilTagTest",group="Explore")
public class AprilTagTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private TimeSplitter durationSplit = TimeSplitter.create("Loop duration");

    // Shape the stick input for more precision at slow speeds:
    public double shapeStick(double stickValue) {
        if (stickValue == 0)
            return 0;
        double result = Math.signum(stickValue) * Math.abs(Math.pow(stickValue, 2.0));
        return result / 2.0 + Math.signum(result) * 0.15;
    }

    @Override
    public void runOpMode() {

        initAprilTag();

        Globals globals = new Globals(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), globals);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData(">", "The quick brown fox jumps over the lazy dog. Now is the time for all good men to come to the aid of their party.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            durationSplit.startSplit();

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            shapeStick(-gamepad1.left_stick_y),
                            shapeStick(-gamepad1.left_stick_x)
                    ),
                    shapeStick(-gamepad1.right_stick_x)
            ));

            drive.updatePoseEstimate();

            // Code added to draw the pose:
            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();
            c.setStroke("#3F51B5");
            MecanumDrive.drawRobot(c, drive.pose);

            // AprilTag logic:
            Pose2d tagPose = processAprilTags(c);
            if (tagPose != null)
                drive.pose = tagPose;

            // Push telemetry to the Driver Station.
            telemetry.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            durationSplit.endSplit();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        TimeSplitter.reportAllResults(); // Look for "TimeSplitter" in Logcat

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

            // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            //.setDrawCubeProjection(false)
            //.setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            // ... these parameters are fx, fy, cx, cy.

                // fx="906.940247073" fy="906.940247073" cx="670.833056673" cy="355.34234068"
                .setLensIntrinsics(906.940247073, 906.940247073, 670.833056673, 355.34234068)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam2"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    static class AprilTagLocation {
        int id;
        double x;
        double y;
        double degrees;
        boolean large;
        AprilTagLocation(int id, double x, double y, double degrees, boolean large) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.degrees = degrees;
            this.large = large;
        }
    }

    /**
     * Process April Tags and return a recommended update to the current pose.
     */
    private Pose2d processAprilTags(Canvas c) {
        Pose2d recommendedPose = null;

        // Camera location on the robot:
        final double CAMERA_OFFSET_X = 8.0; // 0.0;
        final double CAMERA_OFFSET_Y = -5.75; // 8.0;

        AprilTagLocation[] tagLocations = {
                new AprilTagLocation(1,62.875,   42.750, 180, false), // Blue left backdrop, small
                new AprilTagLocation(2,62.875,   36.625, 180, false), // Blue middle backdrop, small
                new AprilTagLocation(3,62.875,   30.625, 180, false), // Blue right backdrop, small
                new AprilTagLocation(4,62.875,  -30.625, 180, false), // Red left backdrop, small
                new AprilTagLocation(5,62.875,  -36.750, 180, false), // Red middle backdrop, small
                new AprilTagLocation(6,62.875,  -42.625, 180, false), // Red right backdrop, small
                new AprilTagLocation(7,  -72, -43.0, 0, true),   // Red audience wall, large
                new AprilTagLocation(8,  -72, -37.5, 0, false),  // Red audience wall, small
                new AprilTagLocation(9,  -72,  37.5, 0, false),  // Blue audience wall, small
                new AprilTagLocation(10, -72,  43.0, 0, true),   // Blue audience wall, large
        };

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                AprilTagLocation tag = tagLocations[0];
                for (AprilTagLocation t: tagLocations) {
                    if (t.id == detection.id)
                        tag = t;
                }

                Pose2d pose;
                double dx = detection.ftcPose.x - CAMERA_OFFSET_X;
                double dy = detection.ftcPose.y - CAMERA_OFFSET_Y;
                double distance = Math.sqrt(dx * dx + dy * dy);

                double gamma = -(Math.atan(dx / dy) + Math.toRadians(detection.ftcPose.yaw) + Math.toRadians(tag.degrees));
                double x = tag.x + Math.cos(gamma) * distance;
                double y = tag.y + Math.sin(gamma) * distance;

                double theta = Math.toRadians(detection.ftcPose.yaw) + Math.toRadians(tag.degrees);
                pose = new Pose2d(new Vector2d(x, y), Math.PI - theta);

                if (tag.large)
                    recommendedPose = pose;

                if (tag.large)
                    c.setStroke("#00ff00");
                else
                    c.setStroke("#c0c000");

                MecanumDrive.drawRobot(c, pose, 7);
                c.strokeCircle(tag.x, tag.y, distance);
            }
        }   // end for() loop

        return recommendedPose;
    }   // end method telemetryAprilTag()

}   // end class
