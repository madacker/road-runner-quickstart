package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "SparkFunOTOS", xmlTag = "SparkFunOTOS")
public class SparkFunOTOS extends I2cDeviceSynchDevice
{
    // Default I2C addresses of the Qwiic OTOS
    final static byte kOtosDefaultAddress = 0x17;

    // OTOS register map
    final static byte kOtosRegProductId = 0x00;
    final static byte kOtosRegHwVersion = 0x01;
    final static byte kOtosRegFwVersion = 0x02;
    final static byte kOtosRegScalarXY = 0x04;
    final static byte kOtosRegScalarH = 0x05;
    final static byte kOtosRegImuCalib = 0x06;
    final static byte kOtosRegReset = 0x07;
    final static byte kOtosRegSelfTest = 0x0E;

    final static byte kOtosRegOffXL = 0x10;
    final static byte kOtosRegOffXH = 0x11;
    final static byte kOtosRegOffYL = 0x12;
    final static byte kOtosRegOffYH = 0x13;
    final static byte kOtosRegOffHL = 0x14;
    final static byte kOtosRegOffHH = 0x15;

    final static byte kOtosRegStatus = 0x1F;

    final static byte kOtosRegPosXL = 0x20;
    final static byte kOtosRegPosXH = 0x21;
    final static byte kOtosRegPosYL = 0x22;
    final static byte kOtosRegPosYH = 0x23;
    final static byte kOtosRegPosHL = 0x24;
    final static byte kOtosRegPosHH = 0x25;
    final static byte kOtosRegVelXL = 0x26;
    final static byte kOtosRegVelXH = 0x27;
    final static byte kOtosRegVelYL = 0x28;
    final static byte kOtosRegVelYH = 0x29;
    final static byte kOtosRegVelHL = 0x2A;
    final static byte kOtosRegVelHH = 0x2B;
    final static byte kOtosRegAccXL = 0x2C;
    final static byte kOtosRegAccXH = 0x2D;
    final static byte kOtosRegAccYL = 0x2E;
    final static byte kOtosRegAccYH = 0x2F;
    final static byte kOtosRegAccHL = 0x30;
    final static byte kOtosRegAccHH = 0x31;

    final static byte kOtosRegPosStdXL = 0x32;
    final static byte kOtosRegPosStdXH = 0x33;
    final static byte kOtosRegPosStdYL = 0x34;
    final static byte kOtosRegPosStdYH = 0x35;
    final static byte kOtosRegPosStdHL = 0x36;
    final static byte kOtosRegPosStdHH = 0x37;
    final static byte kOtosRegVelStdXL = 0x38;
    final static byte kOtosRegVelStdXH = 0x39;
    final static byte kOtosRegVelStdYL = 0x3A;
    final static byte kOtosRegVelStdYH = 0x3B;
    final static byte kOtosRegVelStdHL = 0x3C;
    final static byte kOtosRegVelStdHH = 0x3D;
    final static byte kOtosRegAccStdXL = 0x3E;
    final static byte kOtosRegAccStdXH = 0x3F;
    final static byte kOtosRegAccStdYL = 0x40;
    final static byte kOtosRegAccStdYH = 0x41;
    final static byte kOtosRegAccStdHL = 0x42;
    final static byte kOtosRegAccStdHH = 0x43;

    // Product ID register value
    final static byte kOtosProductId = 0x5F;

    // Conversion factors
    final static double kMeterToInch = 39.37f;
    final static double kInchToMeter = 1.0f / kMeterToInch;
    final static double kRadianToDegree = 180.0f / Math.PI;
    final static double kDegreeToRadian = Math.PI / 180.0f;

    // Conversion factor for X and Y position/velocity/acceleration registers.
    // 16-bit signed registers with a max value of 16 gives a resolution of about
    // 0.00049 meters (0.019 inches)
    final static double kMeterToInt16 = 32768.0f / 16.0f;
    final static double kInt16ToMeter = 1.0f / kMeterToInt16;

    // Conversion factor for the heading register. 16-bit signed register with a
    // max value of pi gives a resolution of about 0.00096 radians (0.0055 degrees)
    final static double kRadToInt16 = 32768.0f / Math.PI;
    final static double kInt16ToRad = 1.0f / kRadToInt16;

    // Conversion factor for the heading velocity register. 16-bit signed register
    // with a max value of 34.9 rps (2000 dps) gives a resolution of about 0.0011
    // rps (0.061 degrees per second)
    final static double kRpsToInt16 = 32768.0f / (2000.0f * kDegreeToRadian);
    final static double kInt16ToRps = 1.0f / kRpsToInt16;

    // Conversion factor for the heading acceleration register. 16-bit signed
    // register with a max value of 3141 rps^2 (180000 dps^2) gives a resolution of
    // about 0.096 rps^2 (5.5 dps^2)
    final static double kRpssToInt16 = 32768.0f / (Math.PI * 1000.0f);
    final static double kInt16ToRpss = 1.0f / kRpssToInt16;

    // Struct to define a 2D pose, including x and y coordinates and an angle h
    public class otos_pose2d_t {
        public double x;
        public double y;
        public double h;

        public otos_pose2d_t(double x, double y, double h)
        {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public otos_pose2d_t()
        {
            this.x = 0;
            this.y = 0;
            this.h = 0;
        }
    }

    enum otos_linear_unit_t {
        kOtosLinearUnitMeters,
        kOtosLinearUnitInches
    }

    enum otos_angular_unit_t {
        kOtosAngularUnitRadians,
        kOtosAngularUnitDegrees
    }

    // Units to be used by the public pose functions. Everything uses meters and
    // radians internally, so this just determines what conversion factor is
    // applied to the public functions
    otos_linear_unit_t _linearUnit;
    otos_angular_unit_t _angularUnit;

    // Conversion factors from meters and radians to the current linear and
    // angular units
    double _meterToUnit;
    double _radToUnit;

    public SparkFunOTOS(I2cDeviceSynchSimple i2cDeviceSynchSimple, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynchSimple, deviceClientIsOwned);

        deviceClient.setI2cAddress(I2cAddr.create7bit(kOtosDefaultAddress));

        // Set default units to inches and degrees
        _linearUnit = otos_linear_unit_t.kOtosLinearUnitInches;
        _angularUnit = otos_angular_unit_t.kOtosAngularUnitDegrees;

        // Set conversion factors to default units
        _meterToUnit = kMeterToInch;
        _radToUnit = kRadianToDegree;
    }

    @Override
    protected boolean doInitialize()
    {
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return null;
    }

    @Override
    public String getDeviceName()
    {
        return null;
    }

    public void calibrateImu()
    {
        calibrateImu(255, true);
    }

    public void calibrateImu(int numSamples, boolean waitUntilDone)
    {
        // Ensure numSamples is an 8-bit value
        numSamples = numSamples & 0xFF;

        // Write the number of samples to the device
        deviceClient.write8(kOtosRegImuCalib, numSamples);

        // Do we need to wait until the calibration finishes?
        if(!waitUntilDone)
            return;

        // Wait for the calibration to finish, which is indicated by the gyro
        // calibration register reading zero
        int calibrationValue = numSamples;
        while(calibrationValue != 0)
        {
            // Give a short delay between reads
            // delayMs(2);

            // Read the gryo calibration register value
            calibrationValue = deviceClient.read8(kOtosRegImuCalib);
        }
    }

    public otos_linear_unit_t getLinearUnit()
    {
        return _linearUnit;
    }

    public void setLinearUnit(otos_linear_unit_t unit)
    {
        // Check if this unit is already set
        if(unit == _linearUnit)
            return;

        // Store new unit
        _linearUnit = unit;

        // Compute conversion factor to new units
        _meterToUnit = (unit == otos_linear_unit_t.kOtosLinearUnitMeters) ? 1.0f : kMeterToInch;
    }

    public otos_angular_unit_t getAngularUnit()
    {
        return _angularUnit;
    }

    public void setAngularUnit(otos_angular_unit_t unit)
    {
        // Check if this unit is already set
        if(unit == _angularUnit)
            return;

        // Store new unit
        _angularUnit = unit;

        // Compute conversion factor to new units
        _radToUnit = (unit == otos_angular_unit_t.kOtosAngularUnitRadians) ? 1.0f : kRadianToDegree;
    }

    public double getLinearScalar()
    {
        // Read the linear scalar from the device
        int rawScalar = (int) deviceClient.read8(kOtosRegScalarXY);

        // Convert to double, multiples of 0.1%
        return (rawScalar * 0.001) + 1.0;
    }

    public void setLinearScalar(double scalar)
    {
        // Check if the scalar is out of bounds, can only be 0.872 to 1.127
        if(scalar < 0.872)
            scalar = 0.872;
        if(scalar > 1.127)
            scalar = 1.127;

        // Convert to raw integer, multiples of 0.1%
        byte rawScalar = (byte)((scalar - 1.0f) * 1000);

        // Write the scalar to the device
        deviceClient.write8(kOtosRegScalarXY, rawScalar);
    }

    public double getAngularScalar()
    {
        // Read the angular scalar from the device
        int rawScalar = (int) deviceClient.read8(kOtosRegScalarH);

        // Convert to double, multiples of 0.1%
        return (rawScalar * 0.001) + 1.0;
    }

    public void setAngularScalar(double scalar)
    {
        // Check if the scalar is out of bounds, can only be 0.872 to 1.127
        if(scalar < 0.872)
            scalar = 0.872;
        if(scalar > 1.127)
            scalar = 1.127;

        // Convert to raw integer, multiples of 0.1%
        byte rawScalar = (byte)((scalar - 1.0f) * 1000);

        // Write the scalar to the device
        deviceClient.write8(kOtosRegScalarH, rawScalar);
    }

    public void resetTracking()
    {
        // Set tracking reset bit
        deviceClient.write8(kOtosRegReset, 0x01);
    }

    public otos_pose2d_t getOffset()
    {
        return readPoseRegs(kOtosRegOffXL, kInt16ToMeter, kInt16ToRad);
    }

    public void setOffset(otos_pose2d_t pose)
    {
        writePoseRegs(kOtosRegOffXL, pose, kMeterToInt16, kRadToInt16);
    }

    public otos_pose2d_t getPosition()
    {
        return readPoseRegs(kOtosRegPosXL, kInt16ToMeter, kInt16ToRad);
    }

    public void setPosition(otos_pose2d_t pose)
    {
        writePoseRegs(kOtosRegPosXL, pose, kMeterToInt16, kRadToInt16);
    }

    public otos_pose2d_t getVelocity()
    {
        return readPoseRegs(kOtosRegVelXL, kInt16ToMeter, kInt16ToRps);
    }

    public otos_pose2d_t getAcceleration()
    {
        return readPoseRegs(kOtosRegAccXL, kInt16ToMeter, kInt16ToRpss);
    }

    public otos_pose2d_t getPositionStdDev()
    {
        return readPoseRegs(kOtosRegPosStdXL, kInt16ToMeter, kInt16ToRad);
    }

    public otos_pose2d_t getVelocityStdDev()
    {
        return readPoseRegs(kOtosRegVelStdXL, kInt16ToMeter, kInt16ToRps);
    }

    public otos_pose2d_t getAccelerationStdDev()
    {
        return readPoseRegs(kOtosRegAccStdXL, kInt16ToMeter, kInt16ToRpss);
    }

    private otos_pose2d_t readPoseRegs(int reg, double rawToXY, double rawToH)
    {
        byte[] rawData = deviceClient.read(reg, 6);

        return regsToPose(rawData, rawToXY, rawToH);
    }

    private void writePoseRegs(int reg, otos_pose2d_t pose, double xyToRaw, double hToRaw)
    {
        // Store raw data in a temporary buffer
        byte[] rawData = poseToRegs(pose, xyToRaw, hToRaw);

        // Write the raw data to the device
        deviceClient.write(reg, rawData);
    }

    private otos_pose2d_t regsToPose(byte[] rawData, double rawToXY, double rawToH)
    {
        int[] rawDataSigned = new int[6];
        for(int i = 0; i < 6; i++)
        {
            rawDataSigned[i] = ((int) rawData[i]) & 0xFF;
        }
        // Store raw data
        int rawX = (rawDataSigned[1] << 8) | rawDataSigned[0];
        int rawY = (rawDataSigned[3] << 8) | rawDataSigned[2];
        int rawH = (rawDataSigned[5] << 8) | rawDataSigned[4];

        if(rawX > 32767)
            rawX -= 65536;
        if(rawY > 32767)
            rawY -= 65536;
        if(rawH > 32767)
            rawH -= 65536;

        otos_pose2d_t pose = new otos_pose2d_t();

        // Store in pose and convert to units
        pose.x = rawX * rawToXY * _meterToUnit;
        pose.y = rawY * rawToXY * _meterToUnit;
        pose.h = rawH * rawToH * _radToUnit;

        return pose;
    }

    private byte[] poseToRegs(otos_pose2d_t pose, double xyToRaw, double hToRaw)
    {
        // Convert pose units to raw data
        int rawX = (int) (pose.x * xyToRaw / _meterToUnit);
        int rawY = (int) (pose.y * xyToRaw / _meterToUnit);
        int rawH = (int) (pose.h * hToRaw / _radToUnit);

        byte[] rawData = new byte[6];

        // Store raw data in buffer
        rawData[0] = (byte) (rawX & 0xFF);
        rawData[1] = (byte) ((rawX >> 8) & 0xFF);
        rawData[2] = (byte) (rawY & 0xFF);
        rawData[3] = (byte) ((rawY >> 8) & 0xFF);
        rawData[4] = (byte) (rawH & 0xFF);
        rawData[5] = (byte) ((rawH >> 8) & 0xFF);

        return rawData;
    }
}
