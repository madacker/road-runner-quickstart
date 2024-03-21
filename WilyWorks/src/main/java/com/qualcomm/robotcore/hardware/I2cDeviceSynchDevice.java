package com.qualcomm.robotcore.hardware;

/**
 * Wily Works custom implementation.
 */
public abstract class I2cDeviceSynchDevice<DEVICE_CLIENT extends I2cDeviceSynch> // implements RobotArmingStateNotifier.Callback, HardwareDevice
{
//    //----------------------------------------------------------------------------------------------
//    // State
//    //----------------------------------------------------------------------------------------------
//
//    protected DEVICE_CLIENT deviceClient;
//    protected boolean       deviceClientIsOwned;
//    protected boolean       isInitialized;
//
//    private static final String TAG = "I2C";
//
//    //----------------------------------------------------------------------------------------------
//    // Construction
//    //----------------------------------------------------------------------------------------------
//
//    protected I2cDeviceSynchDevice(DEVICE_CLIENT deviceClient, boolean deviceClientIsOwned)
//    {
//        this.deviceClient        = deviceClient;
//        this.deviceClientIsOwned = deviceClientIsOwned;
//        this.isInitialized       = false;
//        this.deviceClient.enableWriteCoalescing(false);
//    }
//
//    protected void registerArmingStateCallback(boolean doInitialCallback)
//    {
//        if (deviceClient instanceof RobotArmingStateNotifier)
//        {
//            ((RobotArmingStateNotifier)deviceClient).registerCallback(this, doInitialCallback);
//        }
//    }
//
//    protected void engage()
//    {
//        if (this.deviceClient instanceof Engagable)
//        {
//            ((Engagable)this.deviceClient).engage();
//        }
//    }
//
//    protected void disengage()
//    {
//        if (this.deviceClient instanceof Engagable)
//        {
//            ((Engagable)this.deviceClient).disengage();
//        }
//    }
//
//    //----------------------------------------------------------------------------------------------
//    // Accessing
//    //----------------------------------------------------------------------------------------------
//
//    public DEVICE_CLIENT getDeviceClient()
//    {
//        return this.deviceClient;
//    }
//
//    //----------------------------------------------------------------------------------------------
//    // RobotArmingStateNotifier.Callback
//    //----------------------------------------------------------------------------------------------
//
//    @Override
//    public void onModuleStateChange(RobotArmingStateNotifier module, RobotArmingStateNotifier.ARMINGSTATE state)
//    {
//        // We need to make sure that the actual hardware gets initialized at least once
//        if (state == RobotArmingStateNotifier.ARMINGSTATE.ARMED)
//        {
//            initializeIfNecessary();
//        }
//        else if (state == RobotArmingStateNotifier.ARMINGSTATE.PRETENDING)
//        {
//            // At least make things basically sane
//            initializeIfNecessary();
//            // Next time we arm, we will re-init, as we don't know what's been up with that sensor hw
//            this.isInitialized = false;
//        }
//    }
//
//    protected synchronized void initializeIfNecessary()
//    {
//        if (!this.isInitialized)
//        {
//            RobotLog.ii(TAG, "Automatically initializing I2C device %s (%s)", getClass().getSimpleName(), getConnectionInfo());
//            this.initialize();
//        }
//    }
//
//    public synchronized boolean initialize()
//    {
//        this.isInitialized = this.doInitialize();
//        if (this.isInitialized)
//        {
//            I2cWarningManager.removeProblemI2cDevice(deviceClient);
//        }
//        else
//        {
//            RobotLog.e("Marking I2C device %s %s as unhealthy because initialization failed", getClass().getSimpleName(), getConnectionInfo());
//            I2cWarningManager.notifyProblemI2cDevice(deviceClient);
//        }
//        return this.isInitialized;
//    }
//
//    /**
//     * Actually carries out the initialization of the instance.
//     * @return Whether the initialization was successful or not
//     */
//    protected abstract boolean doInitialize();
//
//    //----------------------------------------------------------------------------------------------
//    // HardwareDevice
//    //----------------------------------------------------------------------------------------------
//
//    @Override
//    public void resetDeviceConfigurationForOpMode()
//    {
//        this.deviceClient.resetDeviceConfigurationForOpMode();
//        this.isInitialized = false;
//        // Instead of performing initialization here (which might take a long time) for a device
//        // that might not even get use, we instead perform initialization when the device is first
//        // retrieved from the HardwareMap.
//    }
//
//    @Override public void close()
//    {
//        if (this.deviceClientIsOwned)
//        {
//            this.deviceClient.close();
//        }
//    }
//
//    @Override
//    public int getVersion()
//    {
//        return 1;
//    }
//
//    @Override
//    public String getConnectionInfo()
//    {
//        return this.deviceClient.getConnectionInfo();
//    }
}
