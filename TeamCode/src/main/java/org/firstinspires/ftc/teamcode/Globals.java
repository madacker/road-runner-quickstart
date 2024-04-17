package org.firstinspires.ftc.teamcode;

import static java.lang.System.nanoTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.wilyworks.common.Wily;
import com.wilyworks.common.WilyWorks;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.inspection.InspectionState;
import org.jetbrains.annotations.Nullable;

/**
 * Set the configuration for Wily Works.
 */
@Wily
class WilyConfig extends WilyWorks.Config {
    WilyConfig() {
        robotWidth = 16;
        robotLength = 18;
        cameras = new Camera[]{
            new Camera("webcamback", -5.75, -6, Math.PI, Math.toRadians(75), 0.190),
            new Camera("webcamfront", 7, -0.5, 0, Math.toRadians(70.4), 0.190)
        };
        distanceSensors = new DistanceSensor[]{
            new DistanceSensor("distance", -1, 3.5, Math.PI)
        };
    }
}

/**
 * Contains all state that can be accessed globally any time within the sensor loop.
 */
public class Globals {
    static final double VOLTAGE_READ_INTERVAL = 0.1; // In seconds, update every 100ms

    // Cached state updated at the start of every sensor loop:
    public static Telemetry telemetry; // Drive Station telemetry
    public static TelemetryPacket packet; // FTC Dashboard telemetry
    public static Canvas canvas; // FTC Dashboard drawing
    public static String botName = getBotName(); // Current robot name
    public static boolean isDevBot = getBotName().equals("DevBot");
    public static boolean isEmulator = getBotName().equals("Emulator");

    // Internal state:
    private int loopCount = 0; // Count of times 'startLoop' has been called
    private static Globals global; // Points to our singleton allocated object
    private Telemetry opmodeTelemetry; // Non-null if initialize() has been called
    private VoltageSensor voltageSensor; // Voltage sensor
    private IMU imu; // Pre-configured IMU object
    private double cachedYaw; // The cached version of the IMU's yaw
    private double cachedVoltage; // The cached voltage reading (amortized)
    private double voltageReadTime; // Time when the voltage sensor was last read

    // Helper to get the robot name:
    private static String getBotName() {
        InspectionState inspection = new InspectionState();
        inspection.initializeLocal();
        return inspection.deviceName;
    }

    // Initialize all of our static state just in case we previously crashed:
    public Globals(HardwareMap hardwareMap, @Nullable Telemetry opmodeTelemetry) {
        this.opmodeTelemetry = opmodeTelemetry;
        if (opmodeTelemetry != null)
            opmodeTelemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Initialize the IMU:
        this.imu = hardwareMap.get(IMU.class, "imu");
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

        // Initialize the voltage sensor:
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Globals.global = this;
        Globals.telemetry = null;
        Globals.packet = null;
        Globals.canvas = null;
    }

    static private double getRawYaw() {
        Stats.startTimer("io::imuYaw");
        Globals.global.cachedYaw = global.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Stats.endTimer("io::imuYaw");
        return Globals.global.cachedYaw;
    }
    static private double getRawRotationRate() {
        Stats.startTimer("io::imuVelocity");
        double rotationRate = global.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        Stats.endTimer("io::imuVelocity");
        return rotationRate;
    }
    static private double getRawVoltage() {
        Stats.startTimer("io::getVoltage");
        Globals.global.cachedVoltage = global.voltageSensor.getVoltage();
        Stats.endTimer("io::getVoltage");
        Stats.addData("Voltage", Globals.global.cachedVoltage);
        return Globals.global.cachedVoltage;
    }
    private void updateCachedState() {
        // Always update the yaw but periodically update the voltage readings:
        getRawYaw();
        double time = time();
        if (time - global.voltageReadTime > VOLTAGE_READ_INTERVAL) {
            global.voltageReadTime = time;
            getRawVoltage();
        }
    }

    // Start of the sensor loop.
    public static void startLoop() {
        if (Globals.global == null) {
            throw new IllegalArgumentException("Forgot to create Globals object!");
        }
        if (Globals.packet != null) {
            throw new IllegalArgumentException("Missing Loop.end() call");
        }

        global.loopCount++;
        global.updateCachedState();

        Globals.telemetry = global.opmodeTelemetry;
        Globals.packet = new TelemetryPacket();
        Globals.canvas = packet.fieldOverlay();
    }

    // End the sensor loop.
    public static void endLoop() {
        if (packet == null) {
            throw new IllegalArgumentException("Missing Loop.start() call");
        }

        Globals.telemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry = null;
        packet = null;
        canvas = null;
    }

    // Return a high resolution time count, in seconds:
    public static double time() {
        return nanoTime() * 1e-9;
    }

    // Normalize an angle to [180, -180):
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle <= -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    // Wrappers for stuff that we cache:
    static public double getYaw() {
        return (global.loopCount > 0) ? global.cachedYaw : getRawYaw();
    }
    static public double getRotationRate() {
        return getRawRotationRate();
    }
    static public double getVoltage() {
        return (global.loopCount > 0) ? global.cachedVoltage : getRawVoltage();
    }

    // Allow direct access for those that need it:
    static public IMU getImu() {
        return global.imu;
    }
    static public VoltageSensor getVoltageSensor() {
        return global.voltageSensor;
    }

    // Assert the Android way:
    static public void assertion(boolean correct, String description) {
        if (!correct) {
            // Congratulations, you've hit one of your own breakpoints! Use the
            // stacktrace to see the context of the culprit!
            throw new AssertionError(description);
        }
    }
    static public void assertion(boolean correct) {
        assertion(correct, "Code assertion");
    }
}
