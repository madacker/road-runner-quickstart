package org.firstinspires.ftc.teamcode.opticaltracking;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;
/**
 * <hr>
 * {@link OpticalTrackingPaa5100} is a driver for the FTC environment for a
 * <a href="https://www.pixart.com/products-detail/74/PAA5100JE-Q">PAA5100</a>
 * optical tracking sensor connected to a hub via a
 * <a href="https://www.nxp.com/docs/en/data-sheet/SC18IS602B.pdf">SC18IS602B</a>
 * I2C to SPI bridge.
 *
 * <p>This code is based on
 * <a href="https://os.mbed.com/teams/PixArt/code/5100_referenceCode/file/782127a132a3/commHeaders/SPIcommFunctions.h/">PixArt reference firmware</a>.
 * </p>
 * <hr>
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
@I2cDeviceType
@DeviceProperties(name = "Optical Tracking Sensor PAA5100/SC18IS602B", description = "Optical Tracking sensor from Marcel", xmlTag = "OPTICAL_TRACKING_PAA5100", compatibleControlSystems = ControlSystem.REV_HUB)
public class OpticalTrackingPaa5100 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // Structure for returning the motion:
    static public class Motion {
        int x;
        int y;
        Motion(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    // The SC18IS602B allows the 3 LSBs of the I2C address to be settable:
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x28);

    // Bitmask that indicates the chip-select of the SC18IS602B SPI/I2C chip that is used for
    // communicating with the attached PAA5100 optical tracking chip:
    public final static int SLAVE_SELECT_MASK = 0x1;

    // I2C register definitions for the SC18IS602B and SPI register definitions for the
    // PAA5100:
    public enum Register {
        // Registers for the SC18IS602B I2C-to-SPI interface:
        I2C_SPI_READ_WRITE0(0x01),
        I2C_SPI_READ_WRITE1(0x02),
        I2C_SPI_READ_WRITE2(0x04),
        I2C_SPI_READ_WRITE3(0x08),
        I2C_CONFIGURE_SPI_INTERFACE(0xf0),
        I2C_CLEAR_INTERRUPT(0xf1),
        I2C_IDLE_MODE(0xf2),
        I2C_GPIO_WRITE(0xf4),
        I2C_GPIO_READ(0xf5),
        I2C_GPIO_ENABLE(0xf6),
        I2C_GPIO_CONFIGURATION(0xf7),

        // Registers for the PMW3901/PAA5100 optical tracking sensor:
        SPI_ID(0x00), // Returns 0x49 for PAA5100
        SPI_DATA_READY(0x02),
        SPI_MOTION_BURST(0x16),
        SPI_POWER_UP_RESET(0x3a),
        SPI_ORIENTATION(0x5b),
        SPI_RESOLUTION(0xe), // PA5100 only
        SPI_RAWDATA_GRAB(0x58),
        SPI_RAWDATA_GRAB_STATUS(0x59);

        public int bVal;
        Register(int bVal) { this.bVal = bVal; }
    }

    public OpticalTrackingPaa5100(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        // Register our device at this I2C address:
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        // Register callback for USB cables getting unplugged:
        super.registerArmingStateCallback(false);

        // We no longer need to change stuff like the I2C address, so engage:
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        // Check chip ID:
        if (readRegister(0x00) != 0x49)
            return false;

        // Do a quick check to see if chip startup succeeded:
        if (!startupCheck())
            return false;

        // Complete the initialization:
        // @@@
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() { return "Optical Tracking Sensor PAA5100/SC18IS602B"; }

    // This function takes an 8-bit address in the form 0x00 and returns an 8-bit value in the
    // form 0x00.
    int readRegister(int addr) {
        // Bus writes:
        //      I2C_ADDRESS
        //      SLAVE_SELECT_MASK
        //      <addr>
        //      0xff (garbage)
        // Bus reads:
        //      I2C_ADDRESS
        //      <addr>
        //      <result>
        byte[] writes = { SLAVE_SELECT_MASK, (byte) addr, (byte) 0xff };
        this.deviceClient.write(writes, I2cWaitControl.WRITTEN);
        byte[] reads = this.deviceClient.read(2);
        return TypeConversion.unsignedByteToInt(reads[1]);
    }

    // This function takes an 8-bit address and 8-bit data. Writes the given data to the given
    // address.
    void writeRegister(int addr, int data) {
        // Write:
        //      I2C_ADDRESS
        //      SLAVE_SELECT_MASK
        //      <addr>
        //      <data>
        byte[] writes = { SLAVE_SELECT_MASK, (byte) addr, (byte) data };
        this.deviceClient.write(writes);
    }

    // Return true if the startup check succeeds, false otherwise:
    boolean startupCheck()
    {
        int startupFail = 0;
        writeRegister(0x7F, 0x00);
        writeRegister(0x55, 0x01);
        writeRegister(0x50, 0x07);
        writeRegister(0x7F, 0x0E);
        writeRegister(0x43, 0x10);

        if(readRegister(0x47) != 0x08) {
            //Checks register 0x47 three times. If the value is incorrect 3 times, throw a fail condition.
            for(int i=0; i<3; i++) {
                if(readRegister(0x47) != 0x08) {
                    writeRegister(0x43, 0x10);
                    startupFail++;
                }
            }
        }
        return startupFail < 3;
    }

    public Motion getMotion() {
        // read/write 13 bytes
        //      0 - unused
        //      1 - dr (unsigned char)
        //      2 - obs (unsigned char)
        //      3 - x (short, little endian)
        //      5 - y (short, little endian)
        //      7 - quality (unsigned char)
        //      8 - raw_sum (unsigned char)
        //      9 - raw_max (unsigned char)
        //      10 - raw_min (unsigned char)
        //      11 - shutter_upper (unsigned char)
        //      12 - shutter_lower (unsigned char)

        int deltaX_low = readRegister(0x03);
        int deltaX_high = (readRegister(0x04)<<8) & 0xFF00;
        int deltaY_low = readRegister(0x05);
        int deltaY_high = (readRegister(0x06)<<8) & 0xFF00;

        int deltaX = deltaX_high | deltaX_low;
        int deltaY = deltaY_high | deltaY_low;

        return new Motion(deltaX, deltaY);
    }
}