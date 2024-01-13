package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public enum MiddleEncoder { FRONT, BACK, LEFT };
    static final public MiddleEncoder MIDDLE_ENCODER = MiddleEncoder.LEFT;
    static final public boolean USE_IMU = true;

    public static class Params {
        public double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 1.0; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)

        Params() {
            switch (MIDDLE_ENCODER) {
                case FRONT:
                    par0YTicks = -7503.0095770759635;
                    par1YTicks = 7314.3664886289025;
                    perpXTicks = 11199.62940916759;
                    break;
                case BACK:
                    par0YTicks = -7369.98756643720;
                    par1YTicks = 7480.191607023889;
                    perpXTicks = -13228.95945230742;
                    break;
                case LEFT:
                    par0YTicks = -13435.250693821241;
                    par1YTicks = 11088.248482224375;
                    perpXTicks = 7472.1530143343225;
                    break;
            }
        }
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private Rotation2d lastHeading;
    private double lastRawHeadingVel, headingVelOffset;

    IMU imu;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, IMU imu, double inPerTick) {
        this.imu = imu;

        switch (MIDDLE_ENCODER) {
            default:
            case FRONT:
                par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFrontMotor-leftEncoder")));
                perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFrontMotor-frontEncoder"))); // Front!
                par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBackMotor-rightEncoder")));
                par1.setDirection(DcMotorSimple.Direction.REVERSE);
                perp.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case BACK:
                par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFrontMotor-leftEncoder")));
                perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBackMotor-backEncoder"))); // Back!
                par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBackMotor-rightEncoder")));
                par1.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case LEFT:
                par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBackMotor-backEncoder")));
                perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFrontMotor-leftEncoder"))); // Left!
                par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFrontMotor-frontEncoder")));
                par1.setDirection(DcMotorSimple.Direction.REVERSE);
                perp.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
        }

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
        lastHeading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
    private double getHeadingVelocity() {
        double rawHeadingVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        return headingVelOffset + rawHeadingVel;
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Vector2dDual vector;
        DualNum<Time> rotational;

        if (MIDDLE_ENCODER == MiddleEncoder.LEFT) {
            // Transform delta-x and delta-y by 90 degrees:
            vector = new Vector2dDual<>(
                            new DualNum<Time>(new double[] {
                                    -(PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                    -(PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                            }).times(inPerTick),
                            new DualNum<Time>(new double[] {
                                    (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                    (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                            }).times(inPerTick)
                    );
        } else {
            // No transformation on delta-x and delta-y:
            vector = new Vector2dDual<>(
                            new DualNum<Time>(new double[]{
                                    (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                    (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                            }).times(inPerTick),
                            new DualNum<Time>(new double[]{
                                    (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                    (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                            }).times(inPerTick)
                    );
        }

        if (USE_IMU) {
            // Use the IMU for orientation:
            Rotation2d heading = Rotation2d.exp(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double headingDelta = heading.minus(lastHeading);
            double headingVel = getHeadingVelocity();
            lastHeading = heading;

            rotational = new DualNum<>(new double[] {
                    headingDelta,
                    headingVel,
            });
        } else {
            // Use the encoders for orientation:
            rotational = new DualNum<>(new double[]{
                    (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                    (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
            });
        }

        Twist2dDual<Time> twist = new Twist2dDual(vector, rotational);

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }
}
