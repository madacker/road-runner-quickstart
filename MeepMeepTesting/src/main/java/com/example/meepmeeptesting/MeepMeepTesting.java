package com.example.meepmeeptesting;

import static java.lang.System.nanoTime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

class MecanumDrive {
    DriveShim shim;
    MecanumDrive(DriveShim shim) {
        this.shim = shim;
    }
    TrajectoryActionBuilder actionBuilder(Pose2d pose) {
        return this.shim.actionBuilder(pose);
    }
}

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new RoadRunnerBotEntity(
                meepMeep,
                // Bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width:
                new Constraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15),
                // Bot dimensions: width, height:
                15, 15,
                new Pose2d(0, 0, 0),
                meepMeep.getColorManager().getTheme(),
                0.8f,
                DriveTrainType.MECANUM,
                false);

        if (true) {
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .splineToLinearHeading(new Pose2d(0, 48, Math.toRadians(-90)), Math.toRadians(90))
                    .endTrajectory()
                    .setTangent(Math.toRadians(-150))
                    .splineToLinearHeading(new Pose2d(0, 24, Math.toRadians(180)), Math.toRadians(-30))
                    .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(-150))
                    .build());
        } else {
            MecanumDrive drive = new MecanumDrive(myBot.getDrive());
            long startTime = nanoTime();

            AutonDriveFactory auton = new AutonDriveFactory(drive);
            Action action = auton.getMeepMeepAction();

            long duration = nanoTime() - startTime;
            double millis = duration / (1000.0 * 1000.0);
            System.out.println(String.format("Milliseconds: %.2f", millis));

            myBot.runAction(action);
        }

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

class AutonDriveFactory {
    public enum PROP_POSITION { LEFT, MIDDLE, RIGHT };

    static public double RED_NEAR_START_OFFSET = 48;

    static public Pose2d START_POSE = new Pose2d(-36, -60, Math.toRadians(90));
    static public double START_TANGENT = Math.toRadians(90);

    static public Pose2d LEFT_HASH_POSE = new Pose2d(-38, -34, Math.toRadians(180));
    static public double LEFT_HASH_STOP_TANGENT = Math.toRadians(180);
    static public double LEFT_HASH_START_TANGENT = Math.toRadians(0);

    static public Pose2d MIDDLE_HASH_POSE_FAR = new Pose2d(-36, -14, Math.toRadians(-90));
    static public Pose2d MIDDLE_HASH_POSE_NEAR = new Pose2d(-36, -34, Math.toRadians(90));
    static public double MIDDLE_HASH_STOP_TANGENT = Math.toRadians(90);
    static public double MIDDLE_HASH_START_TANGENT = Math.toRadians(45);

    static public Pose2d RIGHT_HASH_POSE = new Pose2d(-32, -34, Math.toRadians(0));
    static public double RIGHT_HASH_STOP_TANGENT = Math.toRadians(0);
    static public double RIGHT_HASH_START_TANGENT = Math.toRadians(180);

    static public Vector2d FAR_TRUSS_ENTRANCE_POSITION = new Vector2d(-28, -8);
    static public double FAR_TRUSS_ENTRANCE_TANGENT  = Math.toRadians(0);
    static public Vector2d FAR_TRUSS_EXIT_POSITION = new Vector2d(24, -12);
    static public double FAR_TRUSS_EXIT_TANGENT  = Math.toRadians(0);

    static public Vector2d NEAR_WAYPOINT_POSITION = new Vector2d(12, -54);
    static public double NEAR_WAYPOINT_TANGENT = Math.toRadians(-45);

    static public double BACKBOARD_TANGENT = 0;
    static public Pose2d LEFT_BACKBOARD_POSE = new Pose2d(48, -29, Math.toRadians(180));
    static public Pose2d MIDDLE_BACKBOARD_POSE = new Pose2d(48, -35, Math.toRadians(180));
    static public Pose2d RIGHT_BACKBOARD_POSE = new Pose2d(48, -42, Math.toRadians(180));

    MecanumDrive drive;
    double xformOffsetX = 0;
    double xformMultY = 1.0;

    AutonDriveFactory(MecanumDrive drive) {
        this.drive = drive;
    }

    private Pose2d xform(Pose2d pose) {
        return new Pose2d(
                pose.position.x + xformOffsetX,
                pose.position.y * xformMultY,
                pose.heading.log() * xformMultY);
    }
    private Vector2d xform(Vector2d position) {
        return new Vector2d(position.x + xformOffsetX, position.y * xformMultY);
    }
    private double xform(double tangent) {
        return tangent * xformMultY;
    }
    private PROP_POSITION xform(PROP_POSITION position) {
        if ((xformMultY > 0) || (position == PROP_POSITION.MIDDLE))
            return position;
        return (position == PROP_POSITION.LEFT) ? PROP_POSITION.RIGHT : PROP_POSITION.LEFT;
    }

    Action getCameraAction(boolean red, boolean far, PROP_POSITION position,
                           Action placePurplePixelAction, Action placeBackboardAction) {

        xformOffsetX = far ? 0 : RED_NEAR_START_OFFSET;
        xformMultY = red ? 1 : -1;
        position = xform(position); // Post-transform

        TrajectoryActionBuilder build = this.drive.actionBuilder(xform(START_POSE))
                .setTangent(xform(START_TANGENT));

        // Drive to the prop location:
        if (position == PROP_POSITION.LEFT) {
            build = build.splineToLinearHeading(xform(LEFT_HASH_POSE), xform(LEFT_HASH_STOP_TANGENT));
        } else if (position == PROP_POSITION.MIDDLE) {
            if (far) {
                build = build.splineToLinearHeading(xform(MIDDLE_HASH_POSE_FAR), xform(MIDDLE_HASH_STOP_TANGENT));
            } else {
                build = build.splineToLinearHeading(xform(MIDDLE_HASH_POSE_NEAR), xform(MIDDLE_HASH_STOP_TANGENT));
            }
        } else {
            build = build.splineToLinearHeading(xform(RIGHT_HASH_POSE), xform(RIGHT_HASH_STOP_TANGENT));
        }

        if (placePurplePixelAction != null) {
            build = build.stopAndAdd(placePurplePixelAction);
        } else {
//            Action sleep = new SleepAction(3);
//            build = build.stopAndAdd(sleep);
        }

        xformOffsetX = 0;
        build = build.setReversed(true);
        if (far) {
            build = build.splineTo(xform(FAR_TRUSS_ENTRANCE_POSITION), xform(FAR_TRUSS_ENTRANCE_TANGENT))
                         .splineTo(xform(FAR_TRUSS_EXIT_POSITION), xform(FAR_TRUSS_EXIT_TANGENT));
        } else {
            build = build.splineTo(xform(NEAR_WAYPOINT_POSITION), xform(NEAR_WAYPOINT_TANGENT));
        }

        // Drive to the backboard:
        if (position == PROP_POSITION.LEFT) {
            build = build.splineToSplineHeading(xform(LEFT_BACKBOARD_POSE), xform(BACKBOARD_TANGENT));
        } else if (position == PROP_POSITION.MIDDLE) {
            build = build.splineToSplineHeading(xform(MIDDLE_BACKBOARD_POSE), xform(BACKBOARD_TANGENT));
        } else {
            build = build.splineToSplineHeading(xform(RIGHT_BACKBOARD_POSE), xform(BACKBOARD_TANGENT));
        }

        // Place the yellow pixel on the backboard:
        if (placeBackboardAction != null) {
            build = build.stopAndAdd(placeBackboardAction);
        } else {
            build = build.endTrajectory();
        }

        Vector2d TO_PIXELS_NEAR_POSITION = new Vector2d(12, -60);
        Vector2d TO_PIXELS_FAR_POSITION = new Vector2d(-36, -60);
        double TO_PIXELS_TANGENT = Math.toRadians(180);

        Pose2d WHACK_START_POSE = new Pose2d(-60, -36, Math.toRadians(90));
        Pose2d WHACK_END_POSE = new Pose2d(-60, -24, Math.toRadians(90));
        double WHACK_TANGENT = Math.toRadians(90);

        Vector2d FROM_PIXELS_FAR_POSITION = new Vector2d(-48, -12);
        Vector2d FROM_PIXELS_NEAR_POSITION = new Vector2d(48, -12);
        double FROM_PIXELS_TANGENT = Math.toRadians(0);

        // Drive to knock over the white pixels:
        build = build.setTangent(xform(TO_PIXELS_TANGENT))
                .splineTo(xform(TO_PIXELS_NEAR_POSITION), xform(TO_PIXELS_TANGENT))
                .splineTo(xform(TO_PIXELS_FAR_POSITION), xform(TO_PIXELS_TANGENT))
                .splineToSplineHeading(xform(WHACK_START_POSE), xform(WHACK_TANGENT))
                .splineToSplineHeading(xform(WHACK_END_POSE), xform(WHACK_TANGENT))
                .splineTo(xform(FROM_PIXELS_FAR_POSITION), xform(FROM_PIXELS_TANGENT))
                .splineTo(xform(FROM_PIXELS_NEAR_POSITION), xform(FROM_PIXELS_TANGENT));

        return build.build();
    }

    Action getMeepMeepAction() {
        return getCameraAction(false, false, PROP_POSITION.RIGHT, null, null);
    }
}
