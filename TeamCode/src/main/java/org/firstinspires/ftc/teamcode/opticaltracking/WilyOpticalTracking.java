package org.firstinspires.ftc.teamcode.opticaltracking;

import com.wilyworks.common.WilyWorks;

/**
 * Wily Works version of Optical Tracking.
 */
public class WilyOpticalTracking implements OpticalTracking {
    public WilyOpticalTracking() {
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

    @Override
    public Motion getMotion() {
        return new Motion(0, 0);
    }

    @Override
    public int getQuality() {
        return 0;
    }
}
