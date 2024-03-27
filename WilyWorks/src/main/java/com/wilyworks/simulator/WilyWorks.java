package com.wilyworks.simulator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.wilyworks.simulator.framework.WilyMecanumDrive;
import com.wilyworks.simulator.framework.Simulation;
import com.wilyworks.simulator.framework.WilyTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.reflections.Reflections;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class WilyWorks {
    static final double DELTA_T = 0.100; // 100ms

    static double time() {
        return System.currentTimeMillis() / 1000.0;
    }

    // Structure for representing the choices of opmode:
    static class OpModeChoice {
        Class<?> klass;
        String name;

        public OpModeChoice(Class<?> klass, String name) {
            this.klass = klass; this.name = name;
        }
    }


    /**
     * Enumerate all potential OpModes to be run:
     */
    static List<OpModeChoice> enumerateOpModeChoices() {
        // Use the Reflections library to enumerate all classes in this package that have the
        // @Autonomous and @TeleOp annotations:
        Reflections reflections = new Reflections("org.firstinspires.ftc");
        Set<Class<?>> allOps = new HashSet<>();
        allOps.addAll(reflections.getTypesAnnotatedWith(Autonomous.class));
        allOps.addAll(reflections.getTypesAnnotatedWith(TeleOp.class));
        ArrayList<OpModeChoice> choices = new ArrayList<>();

        // Build a list of the eligible opmodes with their friendly names:
        for (Class klass: allOps) {
            if ((OpMode.class.isAssignableFrom(klass)) &&
                (!klass.isAnnotationPresent(Disabled.class))) {

                // getName() returns a fully qualified name ("org.firstinspires.ftc.teamcode.MyOp").
                // Use only the last portion ("MyOp" in this example):
                String name = klass.getName();
                name = name.substring(name.lastIndexOf(".") + 1); // Skip the dot itself

                // Override the name if an annotation exists:
                TeleOp teleOpAnnotation = (TeleOp) klass.getAnnotation(TeleOp.class);
                if (teleOpAnnotation != null) {
                    if (!teleOpAnnotation.name().equals("")) {
                        name = teleOpAnnotation.name();
                    }
                    if (!teleOpAnnotation.group().equals("")) {
                        name = teleOpAnnotation.group() + ": " + name;
                    }
                }
                Autonomous autonomousAnnotation = (Autonomous) klass.getAnnotation(Autonomous.class);
                if (autonomousAnnotation != null) {
                    if (!autonomousAnnotation.name().equals("")) {
                        name = autonomousAnnotation.name();
                    }
                    if (!autonomousAnnotation.group().equals("")) {
                        name = autonomousAnnotation.group() + ": " + name;
                    }
                }
                choices.add(new OpModeChoice(klass, name));
            }
        }
        return choices;
    }

    static void runOpMode() throws InstantiationException, IllegalAccessException, NoSuchMethodException, InvocationTargetException, InterruptedException {
        List<OpModeChoice> choices = enumerateOpModeChoices();
        Class<?> klass = null;
        for (OpModeChoice choice: choices) {
            if (choice.name.equals("Explore: DistanceTest")) {
                klass = choice.klass;
                break;
            }
        }
        if (klass == null) {
            oldTest();
            return; // ====>
        }

        Object instance = klass.newInstance();
        if (LinearOpMode.class.isAssignableFrom(klass)) {
            LinearOpMode linearOpMode = (LinearOpMode) instance;
            linearOpMode.runOpMode();
        } else {
            // @@@
        }
    }

    static void oldTest() {
        Simulation simulation = new Simulation();
        WilyMecanumDrive mecanumDrive = new WilyMecanumDrive(simulation);
        Gamepad gamepad = new Gamepad();
        WilyTelemetry telemetry = new WilyTelemetry();
        Navigation loop = new Navigation(mecanumDrive, gamepad, telemetry);

        while (true) {
            // We use a fixed time quanta to allow single-stepping in the debugger:
            double time = time();
            gamepad.update();
            loop.update(DELTA_T);
            simulation.update(DELTA_T);

            // Busy-loop to consume any remaining time:
            while (time() - time < DELTA_T)
                ;
        }
    }

    /**
     * Entry point for Wily Works!
     */
    public static void main(String[] args)
    {
        try {
            runOpMode();
        } catch (InstantiationException|IllegalAccessException|NoSuchMethodException|InvocationTargetException e) {
            throw new RuntimeException(e);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}

// Replace this class with your own.
class Navigation {
    WilyMecanumDrive drive;
    Gamepad gamepad1;
    WilyTelemetry telemetry;

    Navigation(WilyMecanumDrive drive, Gamepad gamepad1, WilyTelemetry telemetry) {
        this.drive = drive;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }
    void update(double deltaT) { // Time in seconds
        PoseVelocity2d powers = new PoseVelocity2d(
                new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                -gamepad1.right_stick_x);

        drive.updatePoseEstimate();
        drive.setDrivePowers(powers);

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.update();

        // Code added to draw the pose:
        TelemetryPacket p = new TelemetryPacket();
        Canvas c = p.fieldOverlay();
        c.setStroke("#3F51B5");
        WilyMecanumDrive.drawRobot(c, drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}