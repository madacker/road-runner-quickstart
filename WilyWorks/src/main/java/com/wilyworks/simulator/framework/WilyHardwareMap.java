package com.wilyworks.simulator.framework;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * Wily Works device subclass.
 */
class WilyHardwareDevice implements HardwareDevice {
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
 * Wily Works simulated IMU.
 */
class WilyIMU extends WilyHardwareDevice implements IMU {
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
}

class WilyVoltageSensor extends WilyHardwareDevice implements VoltageSensor {

    @Override
    public double getVoltage() {
        return 13.0;
    }
}

/**
 * Wily Works hardware map.
 */
public class WilyHardwareMap implements Iterable<HardwareDevice> {

    public DeviceMapping<VoltageSensor> voltageSensor = new DeviceMapping<VoltageSensor>(VoltageSensor.class);

    protected Map<String, List<HardwareDevice>> allDevicesMap         = new HashMap<String, List<HardwareDevice>>();
    protected List<HardwareDevice>              allDevicesList        = null;   // cache for iteration

    public WilyHardwareMap() {
        voltageSensor.put("voltage_sensor", new WilyVoltageSensor());
    }

    protected final Object lock = new Object();

    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new LinkedList<T>();
        return result;
    }

    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        if (classOrInterface.equals(IMU.class)) {
            return (T) new WilyIMU();
        }
        return null;
    }

    public HardwareDevice get(String deviceName) { return null; } // @@@


    private void initializeDeviceIfNecessary(HardwareDevice device) {
    }

    private void initializeMultipleDevicesIfNecessary(Iterable<? extends HardwareDevice> devices) {
        for (HardwareDevice device: devices) {
            initializeDeviceIfNecessary(device);
        }
    }

    public SortedSet<String> getAllNames(Class<? extends HardwareDevice> classOrInterface) {
        SortedSet<String> result = new TreeSet<>();
        return result;
    }

    @Override
    public @NonNull Iterator<HardwareDevice> iterator() {
        return new ArrayList<HardwareDevice>().iterator();
    }

    // A DeviceMapping contains a sub-collection of the devices registered in a HardwareMap
    // comprised of all devices of a particular device type.
    public class DeviceMapping<DEVICE_TYPE extends HardwareDevice> implements Iterable<DEVICE_TYPE> {
        private final Map<String, DEVICE_TYPE> map = new HashMap<String, DEVICE_TYPE>();
        private final Class<DEVICE_TYPE> deviceTypeClass;

        public DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass) {
            this.deviceTypeClass = deviceTypeClass;
        }

        public Class<DEVICE_TYPE> getDeviceTypeClass() {
            return this.deviceTypeClass;
        }

        public DEVICE_TYPE cast(Object obj) {
            return this.deviceTypeClass.cast(obj);
        }

        public DEVICE_TYPE get(String deviceName) {
            synchronized (lock) {
                deviceName = deviceName.trim();
                DEVICE_TYPE device = map.get(deviceName);
                if (device == null) {
                    String msg = String.format("Unable to find a hardware device with the name \"%s\"", deviceName);
                    throw new IllegalArgumentException(msg);
                }
                initializeDeviceIfNecessary(device);
                return device;
            }
        }

        public void put(String deviceName, DEVICE_TYPE device) {
            map.put(deviceName.trim(), device);
        }
//
//        public void put(@NonNull SerialNumber serialNumber, String deviceName, DEVICE_TYPE device) {
//            internalPut(serialNumber, deviceName, device);
//        }
//
//        protected void internalPut(@Nullable SerialNumber serialNumber, String deviceName, DEVICE_TYPE device) {
//            synchronized (lock) {
//            }
//        }

        public void putLocal(String deviceName, DEVICE_TYPE device) {
            synchronized (lock) {
            }
        }

        public boolean contains(String deviceName) {
            synchronized (lock) {
                deviceName = deviceName.trim();
                return map.containsKey(deviceName);
            }
        }

        public boolean remove(String deviceName) {
            return remove(null, deviceName);
        }

        public boolean remove(@Nullable SerialNumber serialNumber, String deviceName) {
            synchronized (lock) {
                return false;
            }
        }

        @Override public @NonNull Iterator<DEVICE_TYPE> iterator() {
            synchronized (lock) {
                initializeMultipleDevicesIfNecessary(map.values());
                return new ArrayList<>(map.values()).iterator();
            }
        }

        public Set<Map.Entry<String, DEVICE_TYPE>> entrySet() {
            synchronized (lock) {
                initializeMultipleDevicesIfNecessary(map.values());
                return new HashSet<>(map.entrySet());
            }
        }

        public int size() {
            synchronized (lock) {
                return map.size();
            }
        }
    }
}
