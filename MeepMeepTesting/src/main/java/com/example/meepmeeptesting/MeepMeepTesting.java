package com.example.meepmeeptesting;

import static java.lang.System.nanoTime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
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

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        MecanumDrive drive = new MecanumDrive(myBot.getDrive());
        long startTime = nanoTime();

        //////////////////////////////////////////////////
        Auton auton = new Auton(drive, true, true);
        Action action = auton.getAction();
        //////////////////////////////////////////////////

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

class Auton {
    MecanumDrive drive;
    boolean far;
    boolean red;
    Auton(MecanumDrive drive, boolean far, boolean red) {
        this.drive = drive;
        this.far = far;
        this.red = red;
    }

    Action getAction() {
        return this.drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build();
    }
}
