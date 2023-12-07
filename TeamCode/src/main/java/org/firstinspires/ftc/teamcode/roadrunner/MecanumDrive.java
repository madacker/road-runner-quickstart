package org.firstinspires.ftc.teamcode.roadrunner;

import static java.lang.Math.max;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.jutils.TimeSplitter;
import org.firstinspires.inspection.InspectionState;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {

    public static String getBotName() {
        InspectionState inspection=new InspectionState();
        inspection.initializeLocal();
        Log.d("roadrunner", String.format("Device name:" + inspection.deviceName));
        return inspection.deviceName;
    }
    public static boolean isDevBot = getBotName().equals("DevBot");

    public static class Params {
        public double inPerTick;
        public double lateralInPerTick;
        public double trackWidthTicks;

        public double kS;
        public double kV;
        public double kA;

        public double maxWheelVel;
        public double minProfileAccel;
        public double maxProfileAccel;

        public double maxAngVel;
        public double maxAngAccel;

        public double axialGain;
        public double lateralGain;
        public double headingGain;

        public double axialVelGain;
        public double lateralVelGain;
        public double headingVelGain;

        Params() {
            if (isDevBot) {
                // drive model parameters
                inPerTick = 0.0225669957686882; // 96.0 / 4254.0;
                lateralInPerTick = 0.020179372197309417; // 49.5 / 2453
                trackWidthTicks = 690.3255416844875;

                // feedforward parameters (in tick units)
                kS = 0.6298460597755153;
                kV = 0.004317546531109388;
                kA = 0;

                // path controller gains
                axialGain = 20.0;
                lateralGain = 8.0;
                headingGain = 8.0; // shared with turn
            } else { // Homebot
                // drive model parameters
                inPerTick = 0.000543924757075271254143176860534; // 96 / 176495
                lateralInPerTick = 0.0004651536802430201;
                trackWidthTicks = 26810.36979257466;

                // feedforward parameters (in tick units)
                kS = 0.8528424030378958;
                kV = 0.00007390806402970292;
                kA = 0;

                // path controller gains
                axialGain = 10.0;
                lateralGain = 6.0;
                headingGain = 2.0; // shared with turn
            }

            // path profile parameters (in inches)
            maxWheelVel = 50;
            minProfileAccel = -30;
            maxProfileAccel = 50;

            // turn profile parameters (in radians)
            maxAngVel = Math.PI; // shared with path
            maxAngAccel = Math.PI;

            axialVelGain = 0.0;
            lateralVelGain = 0.0;
            headingVelGain = 0.0; // shared with turn
        }
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final IMU imu;

    public final Localizer localizer;
    public final Localizer localizer2;

    public Pose2d pose;
    public PoseVelocity2d poseVelocity; // Robot-relative, not field-relative
    public Pose2d pose2;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public TimeSplitter loopTime = TimeSplitter.create("Loop time");

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;

        public DriveLocalizer() {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            leftBack.setDirection(DcMotorEx.Direction.REVERSE);

            lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
            lastLeftBackPos = leftBack.getPositionAndVelocity().position;
            lastRightBackPos = rightBack.getPositionAndVelocity().position;
            lastRightFrontPos = rightFront.getPositionAndVelocity().position;

            lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;
        this.pose2 = pose;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (isDevBot) {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

            leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontMotor-leftEncoder");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBackMotor-backEncoder");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBackMotor-rightEncoder");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontMotor-frontEncoder");

            leftBack.setDirection(DcMotorEx.Direction.REVERSE);
            rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        }

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters;
        if (isDevBot) {
            parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        } else {
            parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        }
        imu.initialize(parameters);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        if (isDevBot) {
            localizer = new DriveLocalizer();
            localizer2 = new DriveLocalizer();
        } else {
            localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);
            localizer2 = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick);
        }

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public void setDrivePowers(
            PoseVelocity2d manualPowers, // Can be null
            PoseVelocity2d assistVelocity, // Can be null
            PoseVelocity2d assistAcceleration) { // Yes it's abusing 'PoseVelocity2d' for acceleration

        if (manualPowers == null)
            manualPowers = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (assistVelocity == null)
            assistVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (assistAcceleration == null)
            assistAcceleration = new PoseVelocity2d(new Vector2d(0, 0), 0);

        // Compute the wheel powers for the manual contribution:
        MecanumKinematics.WheelVelocities<Time> manualVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(manualPowers, 1));

        double leftFrontPower = manualVels.leftFront.get(0);
        double leftBackPower = manualVels.leftBack.get(0);
        double rightBackPower = manualVels.rightBack.get(0);
        double rightFrontPower = manualVels.rightFront.get(0);

        // Compute the wheel powers for the assist:
        double[] x = { pose.position.x, assistVelocity.linearVel.x, assistAcceleration.linearVel.x };
        double[] y = { pose.position.y, assistVelocity.linearVel.y, assistAcceleration.linearVel.y };
        double[] angular = { pose.heading.log(), assistVelocity.angVel, assistAcceleration.angVel };

        Pose2dDual<Time> assistDualPose = new Pose2dDual<>(
                new Vector2dDual<>(new DualNum<>(x), new DualNum<>(y)),
                Rotation2dDual.exp(new DualNum<>(angular))
        );

        // Disable the PID part of the PIDF for the assist:
        PoseVelocity2dDual<Time> command = new HolonomicController(0, 0, 0)
                .compute(assistDualPose, pose, poseVelocity);

        MecanumKinematics.WheelVelocities<Time> assistVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();
        final MotorFeedforward feedforward = new MotorFeedforward(
                PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
        );

        // Check for zero velocities and accelerations to avoid adding 'kS'. Arguably these
        // should be epsilon compares but equality is fine for checking when the assist is
        // disabled:
        if ((assistVels.leftFront.get(0) != 0) || (assistVels.leftFront.get(1) != 0))
            leftFrontPower += feedforward.compute(assistVels.leftFront) / voltage;

        if ((assistVels.leftBack.get(0) != 0) || (assistVels.leftBack.get(1) != 0))
            leftBackPower += feedforward.compute(assistVels.leftBack) / voltage;

        if ((assistVels.rightBack.get(0) != 0) || (assistVels.rightBack.get(1) != 0))
            rightBackPower += feedforward.compute(assistVels.rightBack) / voltage;

        if ((assistVels.rightFront.get(0) != 0) || (assistVels.rightFront.get(0) != 0))
            rightFrontPower += feedforward.compute(assistVels.rightFront) / voltage;

        // Normalize if any powers are more than 1:
        double maxPower = max(max(max(max(1, leftFrontPower), leftBackPower), rightBackPower), rightFrontPower);

        leftFront.setPower(leftFrontPower / maxPower);
        leftBack.setPower(leftBackPower / maxPower);
        rightBack.setPower(rightBackPower / maxPower);
        rightFront.setPower(rightFrontPower / maxPower);
    }


    /**
     * Set the drive motor powers according to both user input and automated assist.
     *
     * @param telemetry - For debug spew.
     * @param manual - Motor power as specified by controller input. It's raw power in the range
     *                 of [-1, 1] and in robot-relative coordinates.
     * @param assist - Velocities as specified by automation. It's in ft/s and radians/s and
     *                 in field-relative coordinates.
     */
    public void setAssistedDrivePowersAndUpdatePose(
            Telemetry telemetry,
            PoseVelocity2d manual,
            PoseVelocity2d assist)
    {
        // First calculate the motor voltages considering only the user input. This code is
        // derived from 'setDrivePowers':
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(manual, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value());
        }

        // @@@ Move this normalization to the combined version
        double userLeftFrontV = wheelVels.leftFront.get(0) / maxPowerMag;
        double userLeftBackV = wheelVels.leftBack.get(0) / maxPowerMag;
        double userRightBackV = wheelVels.rightBack.get(0) / maxPowerMag;
        double userRightFrontV = wheelVels.rightFront.get(0) / maxPowerMag;

        // Now calculate the motor voltages as desired by automation. This code is derived
        // from 'FollowTrajectoryAction::run'.
        PoseVelocity2d robotVelRobot = updatePoseEstimate(); // @@@ Move this out so that it can be used?

        double[] xPositionAndVelocity = { pose.position.x, assist.linearVel.x, 0 };
        double[] yPositionAndVelocity = { pose.position.y, assist.linearVel.y, 0 };
        double[] angularHeadingAndVelocity = { pose.heading.log(), assist.angVel, 0 };

        Pose2dDual<Time> targetAutoPose = new Pose2dDual<>(
            new Vector2dDual<>(new DualNum<>(xPositionAndVelocity), new DualNum<>(yPositionAndVelocity)),
            Rotation2dDual.exp(new DualNum<>(angularHeadingAndVelocity))
        );
        PoseVelocity2dDual<Time> command = new HolonomicController(0, 0, 0)
                .compute(targetAutoPose, pose, robotVelRobot);

        wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(
                // @@@ Need to fix 'kS' handling to handle zero velocities!
                // PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
                0, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
        );
        double autoLeftFrontV = feedforward.compute(wheelVels.leftFront) / voltage;
        double autoLeftBackV = feedforward.compute(wheelVels.leftBack) / voltage;
        double autoRightBackV = feedforward.compute(wheelVels.rightBack) / voltage;
        double autoRightFrontV = feedforward.compute(wheelVels.rightFront) / voltage;

        // Set the motors to the combined power:
        leftFront.setPower(userLeftFrontV + autoLeftFrontV);
        leftBack.setPower(userLeftBackV + autoLeftBackV);
        rightBack.setPower(userRightBackV + autoRightBackV);
        rightFront.setPower(userRightFrontV + autoRightFrontV);

        // Some debugging:
        telemetry.addData("Assist X", assist.linearVel.x);
        telemetry.addData("Assist Y", assist.linearVel.y);
        telemetry.addData("User left-front V", userLeftFrontV);
        telemetry.addData("Auto left-front V", autoLeftFrontV);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.log()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#404040");
            drawRobot(c, pose2, 4); // Draw pure odometry pose

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value()); // Draw target pose

            c.setStroke("#3F51B5");
            drawRobot(c, pose); // Draw current pose estimate

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            FlightRecorder.write("TARGET_POSE", new PoseMessage(txWorldTarget.value()));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        loopTime.endSplit();
        loopTime.startSplit();

        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        Twist2dDual<Time> twist2 = localizer2.update(); // Update pure odometry pose
        pose2 = pose2.plus(twist2.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        poseVelocity = twist.velocity().value();
        return poseVelocity;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas c, Pose2d t, double robotRadius) {
        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, robotRadius);

        Vector2d halfv = t.heading.vec().times(0.5 * robotRadius);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;
        drawRobot(c, t, ROBOT_RADIUS);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                0.25, 0.1
        );
    }

    // List of currently running Actions:
    LinkedList<Action> actionList = new LinkedList<>();

    // Invoke an Action to run in parallel during TeleOp:
    public void runParallel(Action action) {
        actionList.add(action);
    }

    // On every iteration of your robot loop, call 'doActionsWork'. Specify the packet
    // if you're drawing on the graph for FTC Dashboard:
    public boolean doActionsWork() { return doActionsWork(null); }
    public boolean doActionsWork(TelemetryPacket packet) {
        LinkedList<Action> deletionList = new LinkedList<>();
        for (Action action: actionList) {
            // Once the Action returns false, the action is done:
            if (!action.run(packet))
                // We can't delete an item from a list while we're iterating on that list:
                deletionList.add(action);
        }
        actionList.removeAll(deletionList);
        return actionList.size() != 0;
    }
}
