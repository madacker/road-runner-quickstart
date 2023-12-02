package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
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

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    boolean USE_BACK_ENCODER = false;

    public static class Params {
        public double par0YTicks = -7503.0095770759635; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 7314.3664886289025; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = 11199.62940916759; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        // Turning loopiness caused by:
        //   o Perpendicular wheel is reversed (when fixed, perpXTicks because positive)
        if (USE_BACK_ENCODER) {
            // We:
            //  o Flipped right and left encoders
            //  o Reversed the *other* parallel encoder
            perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBackMotor-backEncoder"))); // Back!
            par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBackMotor-rightEncoder")));
            par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFrontMotor-leftEncoder")));
            par0.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFrontMotor-frontEncoder"))); // Front!
            par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFrontMotor-leftEncoder")));
            par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBackMotor-rightEncoder")));
            // Road Runner wants the left and right encoders to return negative ticks in the raw view when moving forward!
            // The perpendicular increases as expected though
            par1.setDirection(DcMotorSimple.Direction.REVERSE);
            perp.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }
}
