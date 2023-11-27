package org.firstinspires.ftc.teamcode.tuning;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;
import static com.acmerobotics.roadrunner.Profiles.profile;

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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Wrapper for tracking button press state.
 */
class Buttons {
    final private Gamepad gamepad;
    private boolean a;
    private boolean b;
    private boolean dpad_up;
    private boolean dpad_down;
    Buttons(Gamepad gamepad) {
        this.gamepad = gamepad;
        a = gamepad.a; // @@@
    }
    boolean select() {
        boolean result = a && !gamepad.a;
        a = gamepad.a;
        return result;
    }
    boolean cancel() {
        boolean result = b && !gamepad.b;
        b = gamepad.b;
        return result;
    }
    boolean up() {
        boolean result = dpad_up && !gamepad.dpad_up;
        dpad_up = gamepad.dpad_up;
        return result;
    }
    boolean down() {
        boolean result = dpad_down && !gamepad.dpad_down;
        dpad_down = gamepad.dpad_down;
        return result;
    }
}
@TeleOp
public class RoadRunnerTuner extends LinearOpMode {
    // Member fields referenced by every test:
    Buttons buttons;
    MecanumDrive drive;
    DriveViewFactory driveViewFactory;

    // Constants:
    public static int DISTANCE = 96;
    final Pose2d defaultPose = new Pose2d(0, 0, 0);

    /**
     * Create a DriveViewFactory to call tuning opmodes within road-runner-ftc. Taken from
     * TuningOpModes.register().
     */
    DriveViewFactory getDriveViewFactory(MecanumDrive md) {
        return hardwareMap -> {
            List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
            List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
            if (md.localizer instanceof MecanumDrive.DriveLocalizer) {
                MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) md.localizer;
                leftEncs.add(dl.leftFront);
                leftEncs.add(dl.leftBack);
                rightEncs.add(dl.rightFront);
                rightEncs.add(dl.rightBack);
            } else if (md.localizer instanceof ThreeDeadWheelLocalizer) {
                ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) md.localizer;
                parEncs.add(dl.par0);
                parEncs.add(dl.par1);
                perpEncs.add(dl.perp);
            } else if (md.localizer instanceof TwoDeadWheelLocalizer) {
                TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) md.localizer;
                parEncs.add(dl.par);
                perpEncs.add(dl.perp);
            } else {
                throw new IllegalArgumentException("unknown localizer: " + md.localizer.getClass().getName());
            }

            return new DriveView(
                    DriveType.MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    hardwareMap.getAll(LynxModule.class),
                    Arrays.asList(
                            md.leftFront,
                            md.leftBack
                    ),
                    Arrays.asList(
                            md.rightFront,
                            md.rightBack
                    ),
                    leftEncs,
                    rightEncs,
                    parEncs,
                    perpEncs,
                    md.imu,
                    md.voltageSensor,
                    () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick)
            );
        };
    }

    /**
     * Menu interface for abstracting the menu strings.
     */
    interface MenuStrings {
        String getString(int i);
    }

    /**
     * header is an optional message at the top of the menu.
     * Returns the index of the chosen option or -1 if the cancel button was pressed.
     */
    int menu(String header, int current, boolean topmost, int numStrings, MenuStrings menuStrings) {
        while (opModeIsActive()) {
            if (header != null) {
                telemetry.addLine(header);
            }
            for (int i = 0; i < numStrings; i++) {
                String cursor = (i == current) ? "➤" : "  ";
                telemetry.addLine(cursor + menuStrings.getString(i));
            }
            telemetry.update();
            if (buttons.up()) {
                current--;
                if (current < 0)
                    current = 0;
            }
            if (buttons.down()) {
                current++;
                if (current == numStrings)
                    current = numStrings - 1;
            }
            if (buttons.cancel() && !topmost)
                return -1;
            if (buttons.select())
                return current;
        }
        return topmost ? 0 : -1;
    }

    void showMessage(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }

    boolean readyPrompt(String message) {
        showMessage(message);
        while (opModeIsActive() && !buttons.cancel()) {
            if (buttons.select())
                return true;
        }
        return false;
    }

    void localizerTest() {
        showMessage("Use the controller to drive the robot around. "
                + "Press B to return to the main menu when done.");

        while (opModeIsActive() && !buttons.cancel()) {
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);

            drive.setDrivePowers(powers);
            drive.updatePoseEstimate();

            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();
            c.setStroke("#3F51B5");
            MecanumDrive.drawRobot(c, drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    void lateralInPerTickTuner() {
        if (readyPrompt(String.format("The robot will attempt to strafe left for %d inches. "
            + "Measure the actual distance using a tape measure. "
            + "Multiply 'lateralInPerTick' by <distance-measured> / %d."
            + "\n\nPress A to start", DISTANCE, DISTANCE))) {

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(0, DISTANCE))
                            .build());
        }
    }

    /**
     * This is a re-implementation of 'manualFeedforwardTuner' so that DISTANCE can be changed
     * from its hardcoded 64".
     */
    void manualFeedforwardTuner() {
        if (!readyPrompt(String.format("The robot will attempt to drive forwards then backwards for %d inches. "
                + "Tune 'kV' and 'kS' using FTC Dashboard."
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

        while (opModeIsActive() && !buttons.cancel()) {
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

    void manualFeedbackTunerAxial() {
        if (readyPrompt(String.format("The robot will attempt to drive backwards and forwards for %d inches. "
                + "Tune 'axialGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop", DISTANCE))) {

            while (opModeIsActive() && !buttons.cancel()) {
                Action action = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(DISTANCE)
                        .lineToX(0)
                        .build();

                Actions.runBlocking(action);
            }
        }
    }

    void manualFeedbackTunerLateral() {
        if (readyPrompt(String.format("The robot will attempt to strafe left and right for %d inches. "
                + "Tune 'lateralGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop", DISTANCE))) {

            while (opModeIsActive() && !buttons.cancel()) {
                Action action = drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(0, DISTANCE))
                        .strafeTo(new Vector2d(0, 0))
                        .build();

                Actions.runBlocking(action);
            }
        }
    }

    void manualFeedbackTunerHeading() {
        if (readyPrompt("The robot will attempt to rotate in place "
                + "180 degrees clockwise and counterclockwise. "
                + "Tune 'headingGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop")) {

            while (opModeIsActive() && !buttons.cancel()) {
                Action action = drive.actionBuilder(drive.pose)
                        .turn(Math.PI)
                        .turn(-Math.PI)
                        .build();

                Actions.runBlocking(action);
            }
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
        // Send telemetry to both FTC Dashboard and the Driver Station:
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize member fields:
        drive = new MecanumDrive(hardwareMap, defaultPose);
        buttons = new Buttons(gamepad1);

        // Dynamically build the list of tests:
        ArrayList<Test> tests = new ArrayList<>();
        tests.add(new Test(()->localizerTest(),             "Manual LocalizerTest (drive)"));
        tests.add(new Test(()->lateralInPerTickTuner(),     "Manual lateral tuner (lateralInPerTick)"));
        tests.add(new Test(()->manualFeedforwardTuner(),    "ManualFeedforwardTuner (kV and kS)"));
        tests.add(new Test(()->manualFeedbackTunerAxial(),  "ManualFeedbackTuner (axialGain)"));
        tests.add(new Test(()->manualFeedbackTunerLateral(),"ManualFeedbackTuner (lateralGain)"));
        tests.add(new Test(()->manualFeedbackTunerHeading(),"ManualFeedbackTuner (headingGain)"));

        telemetry.addLine("Press START to begin");
        waitForStart();

        int selection = 0;
        while (opModeIsActive()) {
            selection = menu("Use Dpad to navigate, A to select\n", selection, true,
                    tests.size(), i -> tests.get(i).description);

            tests.get(selection).method.invoke();   // Invoke the chosen test
            drive.pose = defaultPose;               // Reset pose for next test
        }
    }
}
