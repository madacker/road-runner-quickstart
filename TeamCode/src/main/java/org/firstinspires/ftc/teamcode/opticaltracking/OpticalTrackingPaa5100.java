package org.firstinspires.ftc.teamcode.opticaltracking;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.ArrayList;

/**
 * <hr>
 * {@link OpticalTrackingPaa5100} is a driver for the FTC environment for a
 * <a href="https://www.pixart.com/products-detail/74/PAA5100JE-Q">PAA5100</a>
 * optical tracking sensor connected to a hub via a
 * <a href="https://www.nxp.com/docs/en/data-sheet/SC18IS602B.pdf">SC18IS602B</a>
 * I2C to SPI bridge.
 *
 * <p>This code is based on
 * <a href="https://os.mbed.com/teams/PixArt/code/5100_referenceCode/file/782127a132a3/commHeaders/SPIcommFunctions.h/">PixArt reference firmware</a>
 * and <a href="https://github.com/pimoroni/pmw3901-python/tree/master">GitHub python library</a>.
 * </p>
 * <hr>
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
@I2cDeviceType
@DeviceProperties(name = "Optical Tracking Sensor PAA5100/SC18IS602B", description = "Optical Tracking sensor from Marcel", xmlTag = "OPTICAL_TRACKING_PAA5100", compatibleControlSystems = ControlSystem.REV_HUB)
public class OpticalTrackingPaa5100 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // Structure for returning the motion:
    static public class Motion {
        public int x;
        public int y;
        Motion(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    // Special opcode for bulkWrite() encodings:
    private final int WAIT = -1;

    // String tag for logging:
    protected String MYTAG = "PAA5100";

    // The SC18IS602B allows the 3 LSBs of the I2C address to be settable:
    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x28);

    // Bitmask that indicates the chip-select of the SC18IS602B SPI/I2C chip that is used for
    // communicating with the attached PAA5100 optical tracking chip:
    private final static int SLAVE_SELECT_MASK = 0x1;

    // I2C register definitions for the SC18IS602B and SPI register definitions for the
    // PAA5100:
    private enum Register {
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
        SPI_REVISION(0x01), // Returns 0x00 for PAA5100?
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

    //----------------------------------------------------------------------------------------------
    // Configure the SPI interface of the bridge:
    void configureBridge() {
        // Clock rate: 0 = 1843 kHz
        //             1 = 461 kHz
        //             2 = 115 kHz
        //             3 = 58 kHz
        int clockRate = 0;

        // Mode: 0 = SPICLK low when idle, data clocked on leading edge
        //       1 = SPICLK low when idle, data clocked on trailing edge
        //       2 = SPICLK high when idle, data clocked on trailing edge
        //       3 = SPICLK high when idle, data clocked on leading edge
        int mode = 0; // Needs to be idle-low

        // Order: 0 = MSB is first, 1 = LSB is first
        int order = 0;

        int configuration = (order << 5) | (mode << 2) | (clockRate << 0);
        RobotLog.dd(MYTAG, String.format("SPI configuration: 0x%x", configuration));
        this.deviceClient.write8(Register.I2C_CONFIGURE_SPI_INTERFACE.bVal, configuration);
    }

    // This function takes an 8-bit address and returns an 8-bit value:
    int readRegister(int addr) {
        // Write over the bus:
        //      I2C_ADDRESS
        //      SLAVE_SELECT_MASK
        //      <spiAddr>
        //      <0xff> (placeholder)
        // Then read from the bus:
        //      I2C_ADDRESS
        //      <zero> (corresponds to the spiAddr write)
        //      <read-data>

        // Use 0xff as the data placeholder on the write operation:
        this.deviceClient.write(new byte[] { SLAVE_SELECT_MASK, (byte) addr, (byte) 0xff });
        byte[] reads = this.deviceClient.read(2); // address placeholder, datum
        return TypeConversion.unsignedByteToInt(reads[1]); // Skip the address placeholder
    }

    // This function takes an 8-bit address and 8-bit datum. Writes the given datum to the given
    // address.
    void writeRegister(int addr, int datum) {
        // Set the MSB to indicate a write operation:
        this.deviceClient.write(new byte[] { SLAVE_SELECT_MASK, (byte) (addr | 0x80), (byte) datum });
    }

    // Bulk write of SPI register pairs of (address, datum):
    void badBulkWrite(int[] data) {
        ArrayList<Byte> payload = new ArrayList<>();
        for (int dataIndex = 0; ; dataIndex += 2) {
            if ((dataIndex < data.length) && (data[dataIndex] != WAIT)) {
                // Add this pair to the payload, remembering to add a header before the first:
                if (payload.size() == 0) {
                    payload.add((byte) SLAVE_SELECT_MASK);
                }
                payload.add((byte) (data[dataIndex + 0] | 0x80)); // MSB indicates an SPI write
                payload.add((byte) (data[dataIndex + 1]));
            } else {
                // Write all accumulated payload in one blast:
                if (payload.size() != 0) {
                    byte[] writeArray = new byte[payload.size()];
                    for (int i = 0; i < payload.size(); i++)
                        writeArray[i] = payload.get(i);
                    this.deviceClient.write(writeArray);
                }
                if (dataIndex == data.length)
                    return; // ====>
                if (data[dataIndex] == WAIT) {
                    try {
                        sleep(data[dataIndex + 1]);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
                payload.clear();
            }
        }
    }

    // Register write pairs of (address, datum):
    void bulkWrite(int[] data) {
        for (int i = 0; i < data.length; i += 2) {
            if (data[i] == WAIT) {
                try {
                    sleep(data[i + 1]);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            } else {
                writeRegister(data[i], data[i + 1]);
            }
        }
    }

    //----------------------------------------------------------------------------------------------
    // Do the secret-sauce initialization for the PAA5100:
    private void secretSauceInitialization() {
        // The following portion is from __init__()
        // Here we're seeing 0x50 0x0f 0x3x 0x5a
        writeRegister(Register.SPI_POWER_UP_RESET.bVal, 0x5a);
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        for (int i = 0; i < 5; i++) {
            // Here we're seeing:
            //     Write 50, 0f, 02, 00
            //     Read 51, 00, 00, 00 // @@@ Why that length?
            readRegister(Register.SPI_DATA_READY.bVal + i);
        }

        // The following portion is from PAA5100::_secret_sauce():
        bulkWrite(new int[]{
            0x7f, 0x00,
            0x55, 0x01,
            0x50, 0x07,

            0x7f, 0x0e,
            0x43, 0x10
        });
        if ((readRegister(0x67) & 0b10000000) != 0) {
            writeRegister(0x48, 0x04);
        } else {
            writeRegister(0x48, 0x02);
        }
        bulkWrite(new int[]{
            0x7f, 0x00,
            0x51, 0x7b,
            0x50, 0x00,
            0x55, 0x00,
            0x7f, 0x0e
        });
        if (readRegister(0x73) == 0x00) {
            int c1 = readRegister(0x70);
            int c2 = readRegister(0x71);
            if (c1 <= 28) {
                c1 += 14;
            }
            if (c1 > 28) {
                c1 += 11;
            }
            c1 = Math.max(0, Math.min(0x3F, c1));
            c2 = (c2 * 45); // 100
            bulkWrite(new int[]{
                0x7f, 0x00,
                0x61, 0xad,
                0x51, 0x70,
                0x7f, 0x0e
            });
            writeRegister(0x70, c1);
            writeRegister(0x71, c2);
        }
        bulkWrite(new int[]{
            0x7f, 0x00,
            0x61, 0xad,

            0x7f, 0x03,
            0x40, 0x00,

            0x7f, 0x05,
            0x41, 0xb3,
            0x43, 0xf1,
            0x45, 0x14,

            0x5f, 0x34,
            0x7b, 0x08,
            0x5e, 0x34,
            0x5b, 0x11,
            0x6d, 0x11,
            0x45, 0x17,
            0x70, 0xe5,
            0x71, 0xe5,

            0x7f, 0x06,
            0x44, 0x1b,
            0x40, 0xbf,
            0x4e, 0x3f,

            0x7f, 0x08,
            0x66, 0x44,
            0x65, 0x20,
            0x6a, 0x3a,
            0x61, 0x05,
            0x62, 0x05,

            0x7f, 0x09,
            0x4f, 0xaf,
            0x5f, 0x40,
            0x48, 0x80,
            0x49, 0x80,
            0x57, 0x77,
            0x60, 0x78,
            0x61, 0x78,
            0x62, 0x08,
            0x63, 0x50,

            0x7f, 0x0a,
            0x45, 0x60,

            0x7f, 0x00,
            0x4d, 0x11,
            0x55, 0x80,
            0x74, 0x21,
            0x75, 0x1f,
            0x4a, 0x78,
            0x4b, 0x78,
            0x44, 0x08,

            0x45, 0x50,
            0x64, 0xff,
            0x65, 0x1f,

            0x7f, 0x14,
            0x65, 0x67,
            0x66, 0x08,
            0x63, 0x70,
            0x6f, 0x1c,

            0x7f, 0x15,
            0x48, 0x48,

            0x7f, 0x07,
            0x41, 0x0d,
            0x43, 0x14,
            0x4b, 0x0e,
            0x45, 0x0f,
            0x44, 0x42,
            0x4c, 0x80,

            0x7f, 0x10,
            0x5b, 0x02,

            0x7f, 0x07,
            0x40, 0x41,

            WAIT, 0x0a,  // Wait 10ms

            0x7f, 0x00,
            0x32, 0x00,

            0x7f, 0x07,
            0x40, 0x40,

            0x7f, 0x06,
            0x68, 0xf0,
            0x69, 0x00,

            0x7f, 0x0d,
            0x48, 0xc0,
            0x6f, 0xd5,

            0x7f, 0x00,
            0x5b, 0xa0,
            0x4e, 0xa8,
            0x5a, 0x90,
            0x40, 0x80,
            0x73, 0x1f,

            WAIT, 0x0a,  // Wait 10ms

            0x73, 0x00
        });
    }
    // Start and stop the LEDs:
    void setLedState(boolean enable) {
        bulkWrite(new int[]{
            WAIT, 0xF0,
            0x7f, 0x14,
            0x6f, (enable) ? 0x1c : 0x00,
            0x7f, 0x00
        });
    }

    // Initialize the driver:
    @Override
    protected boolean doInitialize() {
        // Configure the SPI interface of the bridge:
        configureBridge();

        // The first thing we should do is check the chip ID because we don't want to
        // send register writes to any random I2C devices:
        int chipId = readRegister(Register.SPI_ID.bVal);
        if (chipId != 0x49) {
            RobotLog.dd(MYTAG, String.format("Bad chip ID (should be 0x49): %x", chipId));
            return false;
        }

        // Initialize the optical tracking chip:
        secretSauceInitialization();

        // Enable the LEDs:
        setLedState(true);

//        // Do a quick check to see if chip startup succeeded:
//        if (!startupCheck())
//            return false;

        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() { return "Optical Tracking Sensor PAA5100/SC18IS602B"; }

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
            // Checks register 0x47 three times. If the value is incorrect 3 times, throw a fail
            // condition.
            for(int i=0; i<3; i++) {
                if(readRegister(0x47) != 0x08) {
                    writeRegister(0x43, 0x10);
                    startupFail++;
                }
            }
        }
        return startupFail < 3;
    }

    // Public API for returning the accumulated motion since the last call:
    public Motion getMotion() {
        final boolean fast = true;
        if (!fast) {
            // Use individual register reads:
            int deltaX_low = readRegister(0x03);
            int deltaX_high = (readRegister(0x04) << 8) & 0xFF00;
            int deltaY_low = readRegister(0x05);
            int deltaY_high = (readRegister(0x06) << 8) & 0xFF00;

            int deltaX = deltaX_high | deltaX_low;
            int deltaY = deltaY_high | deltaY_low;

            return new Motion(deltaX, deltaY);
        } else {
            // Magic minimum quality constant:
            final int MOV_MIN_QUALITY = 0x10;

            // Use burst mode which has a result composed of 13 bytes:
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

            byte[] command = new byte[14]; // Zero initialized by default
            command[0] = SLAVE_SELECT_MASK;
            command[1] = (byte) Register.SPI_MOTION_BURST.bVal;
            for (int i = 2; i < command.length; i++)
                command[i] = (byte) 0xff; // Filler

            this.deviceClient.write(command);

            // -1 because no slave-select mask on read:
            byte[] result = this.deviceClient.read(command.length - 1);

            // Parse the data:
            int dr = TypeConversion.unsignedByteToInt(result[1]);
            int quality = TypeConversion.unsignedByteToInt(result[7]);

            // Note that we switch X and Y to preserve a clockwise coordinate system:
            int deltaY = (TypeConversion.unsignedByteToInt(result[3]))
                       | (result[4] << 8); // Signed conversion
            int deltaX = (TypeConversion.unsignedByteToInt(result[5]))
                       | (result[6] << 8); // Signed conversion

// RobotLog.dd(MYTAG, String.format("dr: 0x%02x, quality: 0x%02x", dr, quality));

            if (((dr & 0b10000000) != 0) && (quality >= MOV_MIN_QUALITY)) {
                return new Motion(deltaX, deltaY);
            } else {
                return new Motion(0, 0);
            }
        }
    }
}
