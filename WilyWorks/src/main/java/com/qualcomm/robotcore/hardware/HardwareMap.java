package com.qualcomm.robotcore.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.SerialNumber;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class HardwareMap {

    protected final Object lock = new Object();

    public DeviceMapping<VoltageSensor>         voltageSensor         = new DeviceMapping<VoltageSensor>(VoltageSensor.class);

    public class DeviceMapping<DEVICE_TYPE extends HardwareDevice> implements Iterable<DEVICE_TYPE> {
        private final Map <String, DEVICE_TYPE> map = new HashMap<String, DEVICE_TYPE>();
        private final Class<DEVICE_TYPE> deviceTypeClass;

        public DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass) {
            this.deviceTypeClass = deviceTypeClass;
        }

        /** Returns the runtime device type for this mapping */
        public Class<DEVICE_TYPE> getDeviceTypeClass() {
            return this.deviceTypeClass;
        }

        /** A small utility that assists in keeping the Java generics type system happy */
        public DEVICE_TYPE cast(Object obj) {
            return this.deviceTypeClass.cast(obj);
        }

        /**
         * Retrieves the device in this DeviceMapping with the indicated name. If no such device is
         * found, an exception is thrown.
         * <p>
         * If the device has not already been initialized, calling this method will initialize it, which
         * may take some time. As a result, you should ONLY call this method during the Init phase of
         * your OpMode.
         *
         * @param deviceName the name of the device object to be retrieved
         * @return a device with the indicated name
         */
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

        /**
         * Registers a new device in this DeviceMapping under the indicated name. Any existing device
         * with this name in this DeviceMapping is removed. The new device is also added to the
         * overall collection in the overall map itself. Note that this method is normally called
         * only by code in the SDK itself, not by user code.
         *
         * @param deviceName  the name by which the new device is to be known (case sensitive)
         * @param device      the new device to be named
         */
        public void put(String deviceName, DEVICE_TYPE device) {
            internalPut(null, deviceName, device);
        }

        /**
         * (Advanced) Registers a new device in this DeviceMapping under the indicated name. Any existing device
         * with this name in this DeviceMapping is removed. The new device is also added to the
         * overall collection in the overall map itself. Note that this method is normally called
         * only by code in the SDK itself, not by user code.
         *
         * @param serialNumber the serial number of the device
         * @param deviceName  the name by which the new device is to be known (case sensitive)
         * @param device      the new device to be named
         */
        public void put(@NonNull SerialNumber serialNumber, String deviceName, DEVICE_TYPE device) {
            internalPut(serialNumber, deviceName, device);
        }

        protected void internalPut(@Nullable SerialNumber serialNumber, String deviceName, DEVICE_TYPE device) {
            synchronized (lock) {
            }
        }

        public void putLocal(String deviceName, DEVICE_TYPE device) {
            synchronized (lock) {
            }
        }

        /**
         * Returns whether a device of the indicated name is contained within this mapping
         * @param deviceName the name sought
         * @return whether a device of the indicated name is contained within this mapping
         */
        public boolean contains(String deviceName) {
            synchronized (lock) {
                deviceName = deviceName.trim();
                return map.containsKey(deviceName);
            }
        }

        /**
         * (Advanced) Removes the device with the indicated name (if any) from this DeviceMapping. The device
         * is also removed under that name in the overall map itself. Note that this method is normally
         * called only by code in the SDK itself, not by user code.
         *
         * @param deviceName  the name of the device to remove.
         * @return            whether any modifications were made to this DeviceMapping
         */
        public boolean remove(String deviceName) {
            return remove(null, deviceName);
        }
        /**
         * (Advanced) Removes the device with the indicated name (if any) from this DeviceMapping. The device
         * is also removed under that name in the overall map itself. Note that this method is normally
         * called only by code in the SDK itself, not by user code.
         *
         * @param serialNumber (optional) the serial number of the device to remove
         * @param deviceName  the name of the device to remove.
         * @return            whether any modifications were made to this DeviceMapping
         */
        public boolean remove(@Nullable SerialNumber serialNumber, String deviceName) {
            synchronized (lock) {
                return false;
            }
        }

        /**
         * Returns an iterator over all the devices in this DeviceMapping. This will initialize any
         * un-initialized devices in the DeviceMapping, so you should ONLY call it during the Init phase
         * of your OpMode.
         *
         * @return an iterator over all the devices in this DeviceMapping.
         */
        @Override public @NonNull Iterator<DEVICE_TYPE> iterator() {
            synchronized (lock) {
                initializeMultipleDevicesIfNecessary(map.values());
                return new ArrayList<>(map.values()).iterator();
            }
        }

        /**
         * Returns a collection of all the (name, device) pairs in this DeviceMapping. This will
         * initialize any un-initialized devices in the DeviceMapping, so you should ONLY call it during
         * the Init phase of your OpMode.
         *
         * @return a collection of all the (name, device) pairs in this DeviceMapping.
         */
        public Set<Map.Entry<String, DEVICE_TYPE>> entrySet() {
            synchronized (lock) {
                initializeMultipleDevicesIfNecessary(map.values());
                return new HashSet<>(map.entrySet());
            }
        }

        /**
         * @return the number of devices currently in this DeviceMapping
         */
        public int size() {
            synchronized (lock) {
                return map.size();
            }
        }
    }

    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new LinkedList<T>();
        return result;
    }

    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        return null;
    }

    private void initializeDeviceIfNecessary(HardwareDevice device) {
    }

    private void initializeMultipleDevicesIfNecessary(Iterable<? extends HardwareDevice> devices) {
        for (HardwareDevice device: devices) {
            initializeDeviceIfNecessary(device);
        }
    }
}
