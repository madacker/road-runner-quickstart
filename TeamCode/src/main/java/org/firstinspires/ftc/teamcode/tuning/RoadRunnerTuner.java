package org.firstinspires.ftc.teamcode.tuning;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;
import static com.acmerobotics.roadrunner.Profiles.profile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

import java.util.ArrayList;
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
    int DISTANCE = 72; // Default to 72 inches for every test
    ArrayList<String> deviceNames = new ArrayList<>();
    Buttons buttons;
    /**
     * header is an optional message at the top of the menu.
     * current is which option to make as the default.
     * options is an array of strings.
     * Returns the index of the chosen option or -1 if the cancel button was pressed.
     */
    int menu(String header, int current, boolean topmost, String[] options) {
        while (isActive()) {
            if (header != null) {
                telemetry.addLine(header);
            }
            for (int i = 0; i < options.length; i++) {
                String cursor = (i == current) ? ">" : " ";
                telemetry.addLine(cursor + options[i]);
            }
            telemetry.update();
            if (buttons.up()) {
                current--;
                if (current < 0)
                    current = options.length - 1;
            }
            if (buttons.down()) {
                current++;
                if (current == options.length)
                    current = 0;
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
        while (isActive() && !buttons.cancel()) {
            if (buttons.select())
                return true;
        }
        return false;
    }

    boolean isActive() {
        return opModeIsActive();
    }

    void localizerTest(MecanumDrive drive) {
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

    void lateralInPerTickTuner(MecanumDrive drive) {
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

    void manualFeedforwardTuner(MecanumDrive drive) {
        // Stolen from TuningOpModes::register

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
        
        // Stolen from ManualFeedforwardTuner::runOpMode()
        //
        // val profile = TimeProfile(constantProfile(
        //   DISTANCE, 0.0, view.maxVel, view.minAccel, view.maxAccel).baseProfile)

        TimeProfile profile = new TimeProfile(constantProfile(
                DISTANCE, 0.0,
                drive.PARAMS.maxWheelVel, drive.PARAMS.minProfileAccel, drive.PARAMS.maxProfileAccel).baseProfile);

        while (isActive() && !buttons.cancel()) {
            // for (i in view.forwardEncsWrapped.indices) {
            //     val v = view.forwardEncsWrapped[i].getPositionAndVelocity().velocity
            //     telemetry.addData("v$i", view.inPerTick * v)
            // }
        }
    }

    void manualFeedbackTunerAxial(MecanumDrive drive) {

    }

    void manualFeedbackTunerLateral(MecanumDrive drive) {
        if (readyPrompt(String.format("The robot will attempt to strafe left and right for %d inches. "
                + "Tune lateralGain to match (values between 1 and 20)."
                + "\n\nPress A to start, B to stop", DISTANCE))) {

            while (isActive() && !buttons.cancel()) {

                Action action = drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(0, DISTANCE))
                        .strafeTo(new Vector2d(0, 0))
                        .build();

                Actions.runBlocking(action);
            }
        }
    }

    void manualFeedbackTunerHeading(MecanumDrive drive) {

    }

    @Override
    public void runOpMode() {
        Pose2d defaultPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, defaultPose);

        telemetry.addLine("Press START to see tuning menu");
        buttons = new Buttons(gamepad1);
        waitForStart();

        int selection = 0;
        while (isActive()) {
            String[] options = {
                    "Manual LocalizerTest (drive)",
                    "Manual lateral tuner (lateralInPerTick)",
//                    "ManualFeedforwardTuner (kV and kS)",
//                    "ManualFeedbackTuner (axial PD)",
                    "ManualFeedbackTuner (lateral PD)",
//                    "ManualFeedbackTuner (heading PD)",
            };
            selection = menu("Use Dpad and A button to select test\n", selection, true, options);
            switch (selection) {
                case 0: localizerTest(drive); break;
                case 1: lateralInPerTickTuner(drive); break;
                case 99997: manualFeedforwardTuner(drive); break;
                case 99998: manualFeedbackTunerAxial(drive); break;
                case 2: manualFeedbackTunerLateral(drive); break;
                case 99999: manualFeedbackTunerHeading(drive); break;
            }
            drive.pose = defaultPose; // Reset pose for next test
        }
    }
}
