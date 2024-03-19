package org.firstinspires.ftc.teamcode.wilyworks;

import android.content.Context;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Iterator;
import java.util.List;
import java.util.Spliterator;
import java.util.function.Consumer;

/**
 * Stub IMU class.
 */
class TestIMU implements IMU {

    @Override
    public boolean initialize(Parameters parameters) {
        return false;
    }

    @Override
    public void resetYaw() {

    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return null;
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return null;
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        return null;
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}

/**
 * Wrap the FTC HardwareMap class.
 */
public class TestHardwareMap extends HardwareMap {
    public TestHardwareMap() {
        super(null, null);
        put("imu", new TestIMU());
    }

    // We can't let the corresponding HardwareMap function handle this call because it references
    // the Device object which raises an error in a static initializer.
    public @Nullable <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        synchronized (lock) {
            deviceName = deviceName.trim();
            List<HardwareDevice> list = allDevicesMap.get(deviceName);
            @Nullable T result = null;

            if (list != null) {
                for (HardwareDevice device : list) {
                    if (classOrInterface.isInstance(device)) {
                        result = classOrInterface.cast(device);
                        break;
                    }
                }
            }
            return result;
        }
    }
}
