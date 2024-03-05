package org.firstinspires.ftc.teamcode.roadrunner;

import static java.lang.Math.max;
import static java.lang.System.nanoTime;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Stats;
import org.firstinspires.inspection.InspectionState;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {

    public static class Params {
        Params() {
            // path profile parameters (in inches and seconds)
            maxWheelVel = 50 / 2;
            minProfileAccel = -30;
            maxProfileAccel = 50;

            // turn profile parameters (in radians)
            maxAngVel = Math.PI / 2.5; // shared with path
            maxAngAccel = Math.PI;

            axialVelGain = 0.0;
            lateralVelGain = 0.0;
            headingVelGain = 0.0; // shared with turn

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
        }

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
    }

    public static String getBotName() {
        InspectionState inspection=new InspectionState();
        inspection.initializeLocal();
        Log.d("roadrunner", String.format("Device name:" + inspection.deviceName));
        return inspection.deviceName;
    }
    public static boolean isDevBot = getBotName().equals("DevBot");

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
    public Pose2d pose;
    public PoseVelocity2d poseVelocity; // Robot-relative, not field-relative

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
            Stats.startTimer("io::getEncoders");
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();
            Stats.endTimer("io::getEncoders");

            Stats.startTimer("io::getImu");
            Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            Stats.endTimer("io::getImu");

            Stats.startTimer("io::encoder?");
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

            Twist2dDual<Time> result = new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );

            Stats.endTimer("io::encoder?");
            return result;
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

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
        } else {
            localizer = new ThreeDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick);
            // localizer = new TwoDeadWheelLocalizer(hardwareMap, imu, PARAMS.inPerTick);
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

    // Used by setDrivePowers to calculate acceleration:
    PoseVelocity2d previousAssistVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    double previousAssistSeconds = 0; // Previous call's nanoTime() in seconds

    /**
     * Power the motors according to the specified velocities. 'stickVelocity' is for controller
     * input and 'assistVelocity' is for computed driver assistance. The former is specified in
     * voltage values normalized from -1 to 1 (just like the regular DcMotor::SetPower() API)
     * whereas the latter is in inches/s or radians/s. Both types of velocities can be specified
     * at the same time in which case the velocities are added together (to allow assist and stick
     * control to blend together, for example).
     *
     * It's also possible to map the controller input to inches/s and radians/s instead of the
     * normalized -1 to 1 voltage range. You can reference MecanumDrive.PARAMS.maxWheelVel and
     * .maxAngVel to determine the range to specify. Note however that the robot can actually
     * go faster than Road Runner's PARAMS values so you would be unnecessarily slowing your
     * robot down.
     */
    public void setDrivePowers(
            Pose2d pose,
            PoseVelocity2d poseVelocity,
            // Manual power, normalized voltage from -1 to 1, robot-relative coordinates, can be null:
            PoseVelocity2d stickVelocity,
            // Computed power, inches/s and radians/s, field-relative coordinates, can be null:
            PoseVelocity2d assistVelocity)
    {
        if (stickVelocity == null)
            stickVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (assistVelocity == null)
            assistVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

        // Compute the assist acceleration as the difference between the new assist velocity
        // and the old divided by delta-t:
        double currentSeconds = nanoTime() * 1e-9;
        PoseVelocity2d assistAcceleration = new PoseVelocity2d(new Vector2d(0, 0), 0);
        if (previousAssistSeconds != 0) {
            double deltaT = currentSeconds - previousAssistSeconds;
            assistAcceleration = new PoseVelocity2d(new Vector2d(
                    (assistVelocity.linearVel.x - previousAssistVelocity.linearVel.x) / deltaT,
                    (assistVelocity.linearVel.y - previousAssistVelocity.linearVel.y) / deltaT),
                    (assistVelocity.angVel - previousAssistVelocity.angVel) / deltaT);
        }
        previousAssistSeconds = currentSeconds;

        // Remember the current velocity for next time:
        previousAssistVelocity = new PoseVelocity2d(new Vector2d(
                assistVelocity.linearVel.x,assistVelocity.linearVel.y), assistVelocity.angVel);

        // Compute the wheel powers for the stick contribution:
        MecanumKinematics.WheelVelocities<Time> manualVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(stickVelocity, 1));

        double leftFrontPower = manualVels.leftFront.get(0);
        double leftBackPower = manualVels.leftBack.get(0);
        double rightBackPower = manualVels.rightBack.get(0);
        double rightFrontPower = manualVels.rightFront.get(0);

        // Compute the wheel powers for the assist:
        double[] x = { pose.position.x, assistVelocity.linearVel.x, assistAcceleration.linearVel.x };
        double[] y = { pose.position.y, assistVelocity.linearVel.y, assistAcceleration.linearVel.y };
        double[] angular = { pose.heading.log(), assistVelocity.angVel, assistAcceleration.angVel };

        Pose2dDual<Time> computedDualPose = new Pose2dDual<>(
                new Vector2dDual<>(new DualNum<>(x), new DualNum<>(y)),
                Rotation2dDual.exp(new DualNum<>(angular)));

        // Compute the feedforward for the assist while disabling the PID portion of the PIDF:
        PoseVelocity2dDual<Time> command = new HolonomicController(0, 0, 0)
                .compute(computedDualPose, pose, poseVelocity);

        MecanumKinematics.WheelVelocities<Time> assistVels = kinematics.inverse(command);

        Stats.startTimer("io::getVoltage");
        double voltage = voltageSensor.getVoltage();
        Stats.endTimer("io::getVoltage");

        final MotorFeedforward feedforward = new MotorFeedforward(
                PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);

        // Check for zero velocities and accelerations to avoid adding 'kS'. Arguably these
        // should be epsilon compares but equality is fine for checking when the assist is
        // disabled:
        if ((assistVels.leftFront.get(0) != 0) || (assistVels.leftFront.get(1) != 0))
            leftFrontPower += feedforward.compute(assistVels.leftFront) / voltage;

        if ((assistVels.leftBack.get(0) != 0) || (assistVels.leftBack.get(1) != 0))
            leftBackPower += feedforward.compute(assistVels.leftBack) / voltage;

        if ((assistVels.rightBack.get(0) != 0) || (assistVels.rightBack.get(1) != 0))
            rightBackPower += feedforward.compute(assistVels.rightBack) / voltage;

        if ((assistVels.rightFront.get(0) != 0) || (assistVels.rightFront.get(1) != 0))
            rightFrontPower += feedforward.compute(assistVels.rightFront) / voltage;

        // Normalize if any powers are more than 1:
        double maxPower = max(max(max(max(1, leftFrontPower), leftBackPower), rightBackPower), rightFrontPower);

        // Set the power to the motors:
        Stats.startTimer("io::setPower");
        leftFront.setPower(leftFrontPower / maxPower);
        leftBack.setPower(leftBackPower / maxPower);
        rightBack.setPower(rightBackPower / maxPower);
        rightFront.setPower(rightFrontPower / maxPower);
        Stats.endTimer("io::setPower");
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

            PoseVelocity2d robotVelRobot = poseVelocity;

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

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("Error (x)", error.position.x);
            p.put("Error (y)", error.position.y);
            p.put("Error (heading) (deg)", Math.toDegrees(error.heading.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value()); // Draw target pose

            // Preview the path:
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

            PoseVelocity2d robotVelRobot = poseVelocity;

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

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public void updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        poseVelocity = twist.velocity().value();
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
    public boolean doActionsWork(Pose2d pose, PoseVelocity2d poseVelocity, TelemetryPacket packet) {
        this.pose = pose;
        this.poseVelocity = poseVelocity;
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

    // Abort all currently running actions:
    public void abortActions() {
        actionList.clear();
    }
}
