package org.firstinspires.ftc.teamcode.tuning;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

class TickTracker {
    enum Correlation {
        FORWARD, REVERSE, ZERO, POSITIVE, NEGATIVE
    }
    enum Mode {
        STRAIGHT, ROTATION
    }
    Mode mode;
    IMU imu;
    double startYaw;

    TickTracker(IMU imu, Mode mode) {
        this.imu = imu;
        this.mode = mode;
        this.startYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    ArrayList<Counter> counters = new ArrayList<>();
    void register(Encoder encoder, String name, Correlation correlation) {
        counters.add(new Counter(encoder, name, correlation));
    }

    double averageTicks() {
        double tickCount = 0;
        double counterCount = 0;
        for (Counter counter: counters) {
            if ((counter.correlation == Correlation.FORWARD) ||
                (counter.correlation == Correlation.REVERSE)) {
                counterCount++;
                tickCount += Math.abs(counter.ticks());
            }
        }
        return tickCount / counterCount;
    }

    boolean report(Telemetry telemetry, String name, String value, String error) {
        boolean passed = true;
        if (!error.equals("")) {
            error = ", " + error;
            passed = false;
        }
        String icon = error.equals("") ? "✅" : "❌";
        telemetry.addLine(String.format("%s <b>%s</b>: %s%s", icon, name, value, error));
        return passed;
    }

    boolean reportAll(Telemetry telemetry) {
        final double ZERO_ERROR = 0.05; // Should be no higher than 5% of the average
        final double STRAIGHT_ERROR = 0.90; // Straight should be no lower than this of the average
        final double ROTATION_ERROR = 0.75; // Rotation should be no lower than this of average

        assert(counters.size() != 0);
        boolean passed = true;
        double averageTicks = averageTicks();

        if (averageTicks < 300) {
            passed &= report(telemetry, "Ticks so far", String.valueOf(averageTicks), "keep pushing");
        } else {
            if (mode == Mode.ROTATION) {
                double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startYaw;
                while (yaw < -180)
                    yaw += 360.0;
                while (yaw >= 180)
                    yaw -= 360.0;

                String error = "";
                if (Math.abs(yaw) < 5) {
                    error = "Yaw too small, are RevHubOrientationOnRobot flags correct when calling imu.initialize()?";
                } else if (yaw < -5) {
                    error = "Yaw is negative, you're turning counterclockwise \uD83D, right?";
                }
                passed &= report(telemetry, "<b>IMU</b>", String.format("%.1f°", yaw), error);
            }

            for (Counter counter : counters) {
                String error = "";
                String value = String.format("%.0f", counter.ticks());
                if (mode == Mode.STRAIGHT) {
                    if (counter.correlation == Correlation.FORWARD) {
                        if (Math.abs(counter.ticks()) < averageTicks * ZERO_ERROR) {
                            error = "is there a bad cable connection?";
                        } else if (counter.ticks() < 0) {
                            error = "set " + counter.name + ".setDirection(DcMotorEx.Direction.REVERSE);";
                        } else if (counter.ticks() < averageTicks * STRAIGHT_ERROR) {
                            error = "too low, does the wheel have bad contact with field?";
                        }
                        value += String.format(" (%.1f%%)", 100*Math.abs(counter.ticks() / averageTicks));
                    } else if (counter.correlation == Correlation.ZERO) {
                        if (Math.abs(counter.ticks()) > averageTicks * 0.05) {
                            error = "should be zero given that it's perpendicular to travel";
                        }
                    }
                } else if (mode == Mode.ROTATION) {
                    if ((counter.correlation == Correlation.FORWARD) ||
                            (counter.correlation == Correlation.REVERSE)) {
                        if (Math.abs(counter.ticks()) < averageTicks * ZERO_ERROR) {
                            error = "is there a bad cable connection?";
                        } else if ((counter.ticks() < 0) && (counter.correlation == Correlation.FORWARD)) {
                            error = "should be positive, are left & right encoders swapped?";
                        } else if ((counter.ticks() > 0) && (counter.correlation == Correlation.REVERSE)) {
                            error = "should be negative, are left & right encoders swapped?";
                        } else if (Math.abs(counter.ticks()) < averageTicks * ROTATION_ERROR) {
                            error = "too low, does the wheel have bad contact with field?";
                        }
                        value += String.format(" (%.1f%%)", 100*Math.abs(counter.ticks() / averageTicks));
                    } else if ((counter.correlation == Correlation.POSITIVE) ||
                            (counter.correlation == Correlation.NEGATIVE)) {
                        if ((counter.ticks() < 0) && (counter.correlation == Correlation.POSITIVE)) {
                            error = "set " + counter.name + ".setDirection(DcMotorEx.Direction.REVERSE);";
                        } else if ((counter.ticks() > 0) && (counter.correlation == Correlation.NEGATIVE)) {
                            error = "set " + counter.name + ".setDirection(DcMotorEx.Direction.REVERSE);";
                        } else if (Math.abs(counter.ticks()) < averageTicks * ZERO_ERROR) {
                            error = "too low, does the wheel have bad contact with field, or "
                                    + "is it too close to center of rotation?";
                        }
                    }
                }
                passed &= report(telemetry, counter.name, value, error);
            }
        }
        return passed;
    }

    class Counter {
        Encoder encoder;
        String name;
        Correlation correlation;
        double initialPosition;
        Counter(Encoder encoder, String name, Correlation correlation) {
            this.encoder = encoder;
            this.name = name;
            this.correlation = correlation;
            this.initialPosition = encoder.getPositionAndVelocity().position;
        }
        double ticks() {
            return encoder.getPositionAndVelocity().position - initialPosition;
        }
    }
}

@TeleOp
public class TuneRoadRunner extends LinearOpMode {
    // Member fields referenced by every test:
    Ui ui;
    MecanumDrive drive;

    // Constants:
    public static int DISTANCE = 72;
    final Pose2d defaultPose = new Pose2d(0, 0, 0);

    // Data structures for the User Interface
    interface MenuStrings {
        String getString(int i);
    }
    class Ui {
        // Button press state:
        private boolean[] buttonPressed = new boolean[4];
        private boolean buttonPress(boolean pressed, int index) {
            boolean press = pressed && !buttonPressed[index];
            buttonPressed[index] = pressed;
            return press;
        }

        // Button press status:
        boolean select() { return buttonPress(gamepad1.a, 0); }
        boolean cancel() { return buttonPress(gamepad1.b, 1); }
        boolean up() { return buttonPress(gamepad1.dpad_up, 2); }
        boolean down() { return buttonPress(gamepad1.dpad_down, 3); }

        // Display the menu:
        int menu(String header, int current, boolean topmost, int numStrings, MenuStrings menuStrings) {
            while (opModeIsActive()) {
                if (up()) {
                    current--;
                    if (current < 0)
                        current = 0;
                }
                if (down()) {
                    current++;
                    if (current == numStrings)
                        current = numStrings - 1;
                }
                if (cancel() && !topmost)
                    return -1;
                if (select())
                    return current;
                if (header != null) {
                    telemetry.addLine(header);
                }
                for (int i = 0; i < numStrings; i++) {
                    String cursor = (i == current) ? "➤" : " ";
                    telemetry.addLine(cursor + menuStrings.getString(i));
                }
                telemetry.update();
                // Sleep to allow other system processing (and ironically improve responsiveness):
                sleep(10);
            }
            return topmost ? 0 : -1;
        }

        // Show a message:
        void showMessage(String message) {
            telemetry.addLine(message);
            telemetry.update();
        }

        // Show a message and wait for A to be pressed:
        boolean readyPrompt(String message) {
            while (opModeIsActive() && !cancel()) {
                showMessage(message);
                if (select())
                    return true;
            }
            return false;
        }
    }

    void pushTest() {
        boolean passed = false;

        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (ui.readyPrompt("Push the robot forward in a straight line for two or more tiles (24\")."
                + "\n\nPress A to start, B when complete")) {

            TickTracker tracker = new TickTracker(drive.imu, TickTracker.Mode.STRAIGHT);
            if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
                MecanumDrive.DriveLocalizer loc = (MecanumDrive.DriveLocalizer) drive.localizer;
                tracker.register(loc.leftFront, "leftFront", TickTracker.Correlation.FORWARD);
                tracker.register(loc.leftBack, "leftBack", TickTracker.Correlation.FORWARD);
                tracker.register(loc.rightBack, "rightBack", TickTracker.Correlation.FORWARD);
                tracker.register(loc.rightFront, "rightFront", TickTracker.Correlation.FORWARD);
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                ThreeDeadWheelLocalizer loc = (ThreeDeadWheelLocalizer) drive.localizer;
                tracker.register(loc.par0, "par0", TickTracker.Correlation.FORWARD);
                tracker.register(loc.par1, "par1", TickTracker.Correlation.FORWARD);
                tracker.register(loc.perp, "perp", TickTracker.Correlation.ZERO);
            } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                TwoDeadWheelLocalizer loc = (TwoDeadWheelLocalizer) drive.localizer;
                tracker.register(loc.par, "par", TickTracker.Correlation.FORWARD);
                tracker.register(loc.perp, "perp", TickTracker.Correlation.ZERO);
            }

            while (opModeIsActive() && !ui.cancel()) {
                // @@@ Show the instructions still
                passed = tracker.reportAll(telemetry);
                if (passed)
                    telemetry.addLine("\nPress B when complete to move to rotation test");
                else
                    telemetry.addLine("\nPress B to cancel");
                telemetry.update();
            }
        }

        if (passed) {
            if (ui.readyPrompt("Rotate the robot 90° counterclockwise \uD83D by pushing."
                    + "\n\nPress A to start, B when complete")) {

                TickTracker tracker = new TickTracker(drive.imu, TickTracker.Mode.ROTATION);
                if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
                    MecanumDrive.DriveLocalizer loc = (MecanumDrive.DriveLocalizer) drive.localizer;
                    tracker.register(loc.leftFront, "leftFront", TickTracker.Correlation.REVERSE);
                    tracker.register(loc.leftBack, "leftBack", TickTracker.Correlation.REVERSE);
                    tracker.register(loc.rightBack, "rightBack", TickTracker.Correlation.FORWARD);
                    tracker.register(loc.rightFront, "rightFront", TickTracker.Correlation.FORWARD);
                } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer loc = (ThreeDeadWheelLocalizer) drive.localizer;
                    tracker.register(loc.par0, "par0", TickTracker.Correlation.REVERSE);
                    tracker.register(loc.par1, "par1", TickTracker.Correlation.FORWARD);
                    tracker.register(loc.perp, "perp", TickTracker.Correlation.POSITIVE);
                } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                    TwoDeadWheelLocalizer loc = (TwoDeadWheelLocalizer) drive.localizer;
                    tracker.register(loc.par, "par", TickTracker.Correlation.REVERSE);
                    tracker.register(loc.perp, "perp", TickTracker.Correlation.POSITIVE);
                }

                while (opModeIsActive() && !ui.cancel()) {
                    tracker.reportAll(telemetry);
                    if (passed)
                        telemetry.addLine("\nPress B when complete");
                    else
                        telemetry.addLine("\nPress B to cancel");
                    telemetry.update();
                }
            }
        }

        // Restore to Road Runner's defaults:
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    void forwardEncoderTuner() {
        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (ui.readyPrompt("Push the robot forward in a straight line as far as possible. "
                + "Measure distance and set inPerTick = <i>inches-traveled</i> / <i>average-ticks</i>."
                + "\n\nPress A to start, B when complete")) {

            TickTracker tracker = new TickTracker(drive.imu, TickTracker.Mode.STRAIGHT);
            if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
                MecanumDrive.DriveLocalizer loc = (MecanumDrive.DriveLocalizer) drive.localizer;
                tracker.register(loc.leftFront, "leftFront", TickTracker.Correlation.FORWARD);
                tracker.register(loc.leftBack, "leftBack", TickTracker.Correlation.FORWARD);
                tracker.register(loc.rightBack, "rightBack", TickTracker.Correlation.FORWARD);
                tracker.register(loc.rightFront, "rightFront", TickTracker.Correlation.FORWARD);
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                ThreeDeadWheelLocalizer loc = (ThreeDeadWheelLocalizer) drive.localizer;
                tracker.register(loc.par0, "par0", TickTracker.Correlation.FORWARD);
                tracker.register(loc.par1, "par1", TickTracker.Correlation.FORWARD);
                tracker.register(loc.perp, "perp", TickTracker.Correlation.ZERO);
            } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                TwoDeadWheelLocalizer loc = (TwoDeadWheelLocalizer) drive.localizer;
                tracker.register(loc.par, "par", TickTracker.Correlation.FORWARD);
                tracker.register(loc.perp, "perp", TickTracker.Correlation.ZERO);
            }

            while (opModeIsActive() && !ui.cancel()) {
                boolean passed = tracker.reportAll(telemetry);
                if (passed) {
                    telemetry.addLine(String.format("\n<b>inPerTick</b> = <i>inches-traveled</i> / %.1f",
                            tracker.averageTicks()));
                }
                telemetry.update();
            }
        }

        // Restore to Road Runner's defaults:
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    void driveTest() {
        while (opModeIsActive() && !ui.cancel()) {
            // @@@ Make it an exponent!
            // @@@ Add control for specific motors!
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);

            drive.setDrivePowers(powers);
            drive.updatePoseEstimate();

            TelemetryPacket p = new TelemetryPacket();
            ui.showMessage("Use the controller to drive the robot around. "
                    + "Press B to return to the main menu when done.");

            Canvas c = p.fieldOverlay();
            c.setStroke("#3F51B5");
            MecanumDrive.drawRobot(c, drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    void lateralEncoderTuner() {
        if (ui.readyPrompt(String.format("The robot will attempt to strafe left for %d inches. "
            + "Measure the actual distance using a tape measure. "
            + "Multiply 'lateralInPerTick' by <distance-measured> / %d."
            + "\n\nPress A to start", DISTANCE, DISTANCE))) {

            runCancelableAction(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(0, DISTANCE))
                            .build());
        }
    }

    // This is a re-implementation of 'manualFeedforwardTuner' so that DISTANCE can be changed
    // from its hardcoded 64".
    void manualFeedforwardTuner() {
        if (!ui.readyPrompt(String.format("The robot will attempt to drive forwards then backwards for %d inches. "
                + "Tune 'kV' and 'kA' using FTC Dashboard."
                + "\n\nPress A to start, B to stop", DISTANCE)))
            return;

        // Taken from TuningOpModes::register:
        List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
        List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
        if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) drive.localizer;
            leftEncs.add(dl.leftFront);
            leftEncs.add(dl.leftBack);
            rightEncs.add(dl.rightFront);
            rightEncs.add(dl.rightBack);
        } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
            ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) drive.localizer;
            parEncs.add(dl.par0);
            parEncs.add(dl.par1);
            perpEncs.add(dl.perp);
        } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
            TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) drive.localizer;
            parEncs.add(dl.par);
            perpEncs.add(dl.perp);
        } else {
            throw new IllegalArgumentException("unknown localizer: " + drive.localizer.getClass().getName());
        }

        List<Encoder> forwardEncsWrapped = new ArrayList<>();
        forwardEncsWrapped.addAll(leftEncs);
        forwardEncsWrapped.addAll(rightEncs);
        forwardEncsWrapped.addAll(parEncs);

        // Everything below here is taken from ManualFeedforwardTuner::runOpMode():
        TimeProfile profile = new TimeProfile(constantProfile(
                DISTANCE, 0.0,
                MecanumDrive.PARAMS.maxWheelVel,
                MecanumDrive.PARAMS.minProfileAccel,
                MecanumDrive.PARAMS.maxProfileAccel).baseProfile);

        boolean movingForwards = true;
        double startTs = System.nanoTime() / 1e9;

        while (opModeIsActive() && !ui.cancel()) {
            TelemetryPacket packet = new TelemetryPacket();

            for (int i = 0; i < forwardEncsWrapped.size(); i++) {
                int v = forwardEncsWrapped.get(i).getPositionAndVelocity().velocity;
                packet.put(String.format("v%d", i), MecanumDrive.PARAMS.inPerTick * v);
            }

            double ts = System.nanoTime() / 1e9;
            double t = ts - startTs;
            if (t > profile.duration) {
                movingForwards = !movingForwards;
                startTs = ts;
            }

            DualNum<Time> v = profile.get(t).drop(1);
            if (!movingForwards) {
                v = v.unaryMinus();
            }
            packet.put("vref", v.get(0));

            MotorFeedforward feedForward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                    MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

            double power = feedForward.compute(v) / drive.voltageSensor.getVoltage();
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0.0), 0.0));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // Set power to zero before exiting:
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
    }

    // Run an Action but end it early if Cancel is pressed.
    // Returns True if it ran without cancelling, False if it was cancelled.
    boolean runCancelableAction(Action action) {
        drive.runParallel(action);
        while (opModeIsActive() && !ui.cancel()) {
            TelemetryPacket packet = new TelemetryPacket();
            ui.showMessage("Press B to stop");
            boolean more = drive.doActionsWork(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            if (!more) {
                // We successfully completed the Action!
                return true; // ====>
            }
        }
        // The user either pressed Cancel or End:
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        return false;
    }

    void manualFeedbackTunerAxial() {
        if (ui.readyPrompt(String.format("The robot will attempt to drive backwards and forwards for %d inches. "
                + "Tune 'axialGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop", DISTANCE))) {

            while (opModeIsActive()) {
                Action action = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(DISTANCE)
                        .lineToX(0)
                        .build();
                if (!runCancelableAction(action))
                    return; // Exit when cancelled
            }
        }
    }

    void manualFeedbackTunerLateral() {
        if (ui.readyPrompt(String.format("The robot will attempt to strafe left and right for %d inches. "
                + "Tune 'lateralGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop", DISTANCE))) {

            while (opModeIsActive()) {
                Action action = drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(0, DISTANCE))
                        .strafeTo(new Vector2d(0, 0))
                        .build();
                if (!runCancelableAction(action))
                    return; // Exit when cancelled
            }
        }
    }

    void manualFeedbackTunerHeading() {
        if (ui.readyPrompt("The robot will attempt to rotate in place "
                + "180° clockwise and counterclockwise. "
                + "Tune 'headingGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop")) {

            while (opModeIsActive()) {
                Action action = drive.actionBuilder(drive.pose)
                        .turn(Math.PI)
                        .turn(-Math.PI)
                        .build();
                if (!runCancelableAction(action))
                    return; // Exit when cancelled
            }
        }
    }

    void completionTest() {
        if (ui.readyPrompt("The robot will drive forward 48 inches using a spline. "
                + "It needs half a tile clearance on either side. "
                + "\n\nPress A to start, B to stop")) {

            Action action = drive.actionBuilder(drive.pose)
                    .setTangent(Math.toRadians(60))
                    .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(90)), Math.toRadians(-60))
                    .splineToLinearHeading(new Pose2d(48, 0, Math.toRadians(180)), Math.toRadians(60))
                    .endTrajectory()
                    .setTangent(Math.toRadians(-180))
                    .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-0.0001)), Math.toRadians(-180))
                    .build();
            runCancelableAction(action);
        }
    }

    // Data structures for building a table of tests:
    interface TestMethod {
        void invoke();
    }
    class Test {
        TestMethod method;
        String description;
        public Test(TestMethod method, String description) {
            this.method = method;
            this.description = description;
        }
    }

    @Override
    public void runOpMode() {
        // Set the display format to use HTML:
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Send telemetry to both FTC Dashboard and the Driver Station:
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize member fields:
        drive = new MecanumDrive(hardwareMap, defaultPose);
        ui = new Ui();

        // Dynamically build the list of tests:
        ArrayList<Test> tests = new ArrayList<>();
        tests.add(new Test(this::driveTest,                 "Drive test (motors)"));
        tests.add(new Test(this::pushTest,                  "Push test (encoders)"));
        tests.add(new Test(this::forwardEncoderTuner,       "Forward encoder tuner (inPerTick)"));
        // @@@ Call lateralEncoderTuner only if no dead wheels:
        tests.add(new Test(this::lateralEncoderTuner,       "Lateral encoder tuner (lateralInPerTick)"));
        tests.add(new Test(this::manualFeedforwardTuner,    "ManualFeedforwardTuner (kV and kA)"));
        tests.add(new Test(this::manualFeedbackTunerAxial,  "ManualFeedbackTuner (axialGain)"));
        tests.add(new Test(this::manualFeedbackTunerLateral,"ManualFeedbackTuner (lateralGain)"));
        tests.add(new Test(this::manualFeedbackTunerHeading,"ManualFeedbackTuner (headingGain)"));
        tests.add(new Test(this::completionTest,            "Completion test (verification)"));

        telemetry.addLine("Press START to begin");
        waitForStart();

        int selection = 0;
        while (opModeIsActive()) {
            selection = ui.menu("Use Dpad to navigate, A to select\n", selection, true,
                    tests.size(), i -> tests.get(i).description);

            tests.get(selection).method.invoke();   // Invoke the chosen test
            drive.pose = defaultPose;               // Reset pose for next test
        }
    }
}
