package com.example.meepmeeptesting;

import static java.lang.System.nanoTime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
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

        MecanumDrive drive = new MecanumDrive(myBot.getDrive());
        long startTime = nanoTime();

        AutonDriveFactory auton = new AutonDriveFactory(drive);
        Action action = auton.getMeepMeepAction();

        long duration = nanoTime() - startTime;
        double millis = duration / (1000.0 * 1000.0);
        System.out.println(String.format("Milliseconds: %.2f", millis));

        myBot.runAction(action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

class AutonDriveFactory {
    public enum POSE_POSITION { LEFT, MIDDLE, RIGHT };

    static public double RED_NEAR_START_OFFSET = 48;

    static public Pose2d START_POSE = new Pose2d(-36, -60, Math.toRadians(90));
    static public double START_TANGENT = Math.toRadians(90);

    static public Pose2d LEFT_HASH_POSE = new Pose2d(-38, -34, Math.toRadians(180));
    static public double LEFT_HASH_TANGENT = Math.toRadians(180);

    static public Pose2d MIDDLE_HASH_POSE = new Pose2d(-36, -14, Math.toRadians(-90));
    static public double MIDDLE_HASH_ENTER_TANGENT = Math.toRadians(90);
    static public double MIDDLE_HASH_LEAVE_TANGENT = Math.toRadians(45);

    static public Pose2d RIGHT_HASH_POSE = new Pose2d(-32, -34, Math.toRadians(0));
    static public double RIGHT_HASH_TANGENT = Math.toRadians(0);

    static public Vector2d FAR_TRUSS_ENTRANCE_POSITION = new Vector2d(-24, -12);
    static public double FAR_TRUSS_ENTRANCE_TANGENT  = Math.toRadians(0);
    static public Vector2d FAR_TRUSS_EXIT_POSITION = new Vector2d(24, -12);
    static public double FAR_TRUSS_EXIT_TANGENT  = Math.toRadians(0);

    static public Vector2d NEAR_WAYPOINT_POSITION = new Vector2d(12, -54);
    static public double NEAR_WAYPOINT_TANGENT = Math.toRadians(-45);

    static public double BACKBOARD_TANGENT = 0;
    static public Pose2d LEFT_BACKBOARD_POSE = new Pose2d(48, -30, Math.toRadians(180));
    static public Pose2d MIDDLE_BACKBOARD_POSE = new Pose2d(48, -36, Math.toRadians(180));
    static public Pose2d RIGHT_BACKBOARD_POSE = new Pose2d(48, -42, Math.toRadians(180));

    MecanumDrive drive;
    boolean isRed;
    boolean isFar;
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

    Action getCameraAction(boolean red, boolean far, POSE_POSITION position,
                           Action placePurpleAction, Action placeBackdropAction) {
        xformOffsetX = far ? 0 : RED_NEAR_START_OFFSET;
        xformMultY = red ? 1 : -1;

        TrajectoryActionBuilder build = this.drive.actionBuilder(xform(START_POSE))
                .setTangent(xform(START_TANGENT));

        if (position == POSE_POSITION.LEFT) {
            build = build.splineToSplineHeading(xform(LEFT_HASH_POSE), xform(LEFT_HASH_TANGENT));
        } else if (position == POSE_POSITION.MIDDLE) {
            build = build.splineToSplineHeading(xform(MIDDLE_HASH_POSE), xform(MIDDLE_HASH_ENTER_TANGENT));
        } else {
            build = build.splineToSplineHeading(xform(RIGHT_HASH_POSE), xform(RIGHT_HASH_TANGENT));
        }

        if (placePurpleAction != null) {
            build = build.stopAndAdd(placePurpleAction);
        } else {
            Action sleep = new SleepAction(3);
            build = build.stopAndAdd(sleep);
        }

        xformOffsetX = 0;

        if (far) {
            if (position == POSE_POSITION.LEFT) {
                build = build.setTangent(xform(LEFT_HASH_TANGENT));
            } else if (position == POSE_POSITION.MIDDLE) {
                build = build.setTangent(xform(MIDDLE_HASH_LEAVE_TANGENT));
            } else {
                build = build.setTangent(xform(RIGHT_HASH_TANGENT));
            }
            build = build.splineToConstantHeading(xform(FAR_TRUSS_ENTRANCE_POSITION), xform(FAR_TRUSS_ENTRANCE_TANGENT))
                    .splineToConstantHeading(xform(FAR_TRUSS_EXIT_POSITION), xform(FAR_TRUSS_EXIT_TANGENT));
        } else {
            if (position == POSE_POSITION.LEFT) {
                build = build.setTangent(xform(Math.PI - LEFT_HASH_TANGENT));
            } else if (position == POSE_POSITION.MIDDLE) {
                build = build.setTangent(xform(Math.PI - MIDDLE_HASH_ENTER_TANGENT));
            } else {
                build = build.setTangent(xform(Math.PI - RIGHT_HASH_TANGENT));
            }
            build = build.splineToConstantHeading(xform(NEAR_WAYPOINT_POSITION), xform(NEAR_WAYPOINT_TANGENT));
        }

        if (position == POSE_POSITION.LEFT) {
            build = build.splineToSplineHeading(xform(LEFT_BACKBOARD_POSE), xform(BACKBOARD_TANGENT));
        } else if (position == POSE_POSITION.MIDDLE) {
            build = build.splineToSplineHeading(xform(MIDDLE_BACKBOARD_POSE), xform(BACKBOARD_TANGENT));
        } else {
            build = build.splineToSplineHeading(xform(RIGHT_BACKBOARD_POSE), xform(BACKBOARD_TANGENT));
        }

        return build.build();
    }

    Action getMeepMeepAction() {
        return getCameraAction(true, false, POSE_POSITION.RIGHT, null, null);
    }
}
