"""This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.
Made by: MrTijn/Tijndagamer
Released under the MIT License
Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer
"""

import smbus2
import time

class mpu6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # Config Register definitions : Page 13 https://store.invensense.com/Datasheets/invensense/RM-MPU-6000A.pdf
    CONFIG_EXT_SYNC_SET_BIT = 5
    CONFIG_EXT_SYNC_SET_LENGTH = 3
    CONFIG_DLPF_CFG_BIT = 2
    CONFIG_DLPF_CFG_LENGTH = 3

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    CONFIG = 0x1A
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus2.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        # and set clock to PLL w/ x-axis gyro
        # Page 41 https://store.invensense.com/Datasheets/invensense/RM-MPU-6000A.pdf
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x01)

        # // Configure Gyro and Accelerometer
        # // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
        # // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
        self.bus.write_byte_data(self.address, self.CONFIG, 0x03)  # configuration

        self.set_accel_range(self.ACCEL_RANGE_4G)
        self.set_gyro_range(self.GYRO_RANGE_250DEG)

    # Deleting (Calling destructor)
    def __del__(self):
        self.bus.close()

    # I2C communication methods
    # Todo: move these to an extended smbus2 implementation

    def read_i2c_word_data(self, register):
        tries = 0
        while True:
            try:

                tries = tries + 1

                """Read two i2c registers and combine them.
                
                        register -- the first register to read from.
                        Returns the combined read results.
                        """
                # Read the data from the registers
                high = self.bus.read_byte_data(self.address, register)
                low = self.bus.read_byte_data(self.address, register + 1)

                value = (high << 8) + low

                if value >= 0x8000:
                    return -((65535 - value) + 1)
                else:
                    return value

            except IOError:
                if tries > 4:
                    raise
                else:
                    time.sleep(.250)

    def read_bits(self, dev_addr, reg_addr, bit_start, length):
        """ Read multiple bits from an 8-bit device register.

        :param dev_addr: I2C slave device address
        :param reg_addr: Register regAddr to read from
        :param bit_start: First bit position to read (0-7)
        :param length: length Number of bits to read (not more than 8)
        :return: Read byte value
        :rtype: int
        """

        raw_data = self.bus.read_byte_data(dev_addr, reg_addr)

        mask = ((1 << length) - 1) << (bit_start - length + 1)
        raw_data &= mask
        raw_data >>= (bit_start - length + 1)
        return raw_data

    def write_bits(self, dev_addr, reg_addr, bit_start, length, data):
        """ Write multiple bits in an 8-bit device register.

        :param dev_addr: I2C slave device address
        :param reg_addr: Register regAddr to write to
        :param bit_start: First bit position to write (0-7)
        :param length: Number of bits to write (not more than 8)
        :param data: Right-aligned value to write
        """
        # 010 value to write
        # 76543210 bit numbers
        # xxx args: bitStart = 4, length = 3
        # 00011100 mask byte
        # 10101111 original value(sample)
        # 10100011 original & ~mask
        # 10101011 masked | value

        # get current value of register
        raw_data = self.bus.read_byte_data(dev_addr, reg_addr)

        mask = ((1 << length) - 1) << (bit_start - length + 1)
        data <<= (bit_start - length + 1)  # shift data into correct position
        data &= mask  # zero all non-important bits in data
        raw_data &= ~mask  # zero all important bits in existing byte
        raw_data |= data  # combine data with existing byte
        self.bus.write_byte_data(dev_addr, reg_addr, raw_data)

    # MPU-6050 Methods

    def get_external_frame_sync(self):
        """ Get external FSYNC configuration.

          Configures the external Frame Synchronization (FSYNC) pin sampling. An
          external signal connected to the FSYNC pin can be sampled by configuring
          EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
          strobes may be captured. The latched FSYNC signal will be sampled at the
          Sampling Rate, as defined in register 25. After sampling, the latch will
          reset to the current FSYNC signal state.

          The sampled value will be reported in place of the least significant bit in
          a sensor data register determined by the value of EXT_SYNC_SET according to
          the following table.

          <pre>
          EXT_SYNC_SET | FSYNC Bit Location
          -------------+-------------------
          0            | Input disabled
          1            | TEMP_OUT_L[0]
          2            | GYRO_XOUT_L[0]
          3            | GYRO_YOUT_L[0]
          4            | GYRO_ZOUT_L[0]
          5            | ACCEL_XOUT_L[0]
          6            | ACCEL_YOUT_L[0]
          7            | ACCEL_ZOUT_L[0]
          </pre>
        :return: FSYNC configuration value
        """
        return self.read_bits(self.address, self.CONFIG, self.CONFIG_EXT_SYNC_SET_BIT, self.CONFIG_EXT_SYNC_SET_LENGTH)

    def set_external_frame_sync(self, sync):
        """ Set external FSYNC configuration.

          EXT_SYNC_SET | FSYNC Bit Location
          -------------+-------------------
          0            | Input disabled
          1            | TEMP_OUT_L[0]
          2            | GYRO_XOUT_L[0]
          3            | GYRO_YOUT_L[0]
          4            | GYRO_ZOUT_L[0]
          5            | ACCEL_XOUT_L[0]
          6            | ACCEL_YOUT_L[0]
          7            | ACCEL_ZOUT_L[0]

        :param sync: value to set for sync (0-7)
        """
        self.write_bits(self.address, self.CONFIG, self.CONFIG_EXT_SYNC_SET_BIT, self.CONFIG_EXT_SYNC_SET_LENGTH, sync)

    def get_dlpf_mode(self):
        """  Get digital low-pass filter configuration.

          The DLPF_CFG parameter sets the digital low pass filter configuration. It
          also determines the internal sampling rate used by the device as shown in
          the table below.

          Note: The accelerometer output rate is 1kHz. This means that for a Sample
          Rate greater than 1kHz, the same accelerometer sample may be output to the
          FIFO, DMP, and sensor registers more than once.

          <pre>
                   |   ACCELEROMETER    |           GYROSCOPE
          DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
          ---------+-----------+--------+-----------+--------+-------------
          0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
          1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
          2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
          3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
          4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
          5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
          6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
          7        |   -- Reserved --   |   -- Reserved --   | Reserved
          </pre>
        :return: DLFP configuration
        """
        return self.read_bits(self.address, self.CONFIG, self.CONFIG_DLPF_CFG_BIT, self.CONFIG_DLPF_CFG_LENGTH)

    def set_dlpf_mode(self, mode):
        """ Set digital low-pass filter configuration.

                   |   ACCELEROMETER    |           GYROSCOPE
          DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
          ---------+-----------+--------+-----------+--------+-------------
          0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
          1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
          2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
          3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
          4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
          5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
          6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
          7        |   -- Reserved --   |   -- Reserved --   | Reserved

        :param mode: value to set for dlpf mode (0-6)
        :return:
        """
        self.write_bits(self.address, self.CONFIG, self.CONFIG_DLPF_CFG_BIT, self.CONFIG_DLPF_CFG_LENGTH, mode)

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.

        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word_data(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.

        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.

        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False):
        """Gets and returns the X, Y and Z values from the accelerometer.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word_data(self.ACCEL_XOUT0)
        y = self.read_i2c_word_data(self.ACCEL_YOUT0)
        z = self.read_i2c_word_data(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range! {} - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G".format(accel_range))
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.

        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to.

        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.

        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word_data(self.GYRO_XOUT0)
        y = self.read_i2c_word_data(self.GYRO_YOUT0)
        z = self.read_i2c_word_data(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range! {} - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG".format(gyro_range))
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return [accel, gyro, temp]


if __name__ == "__main__":
    mpu = mpu6050(0x68)
    print(mpu.get_temp())
    accel_data = mpu.get_accel_data()
    print(accel_data['x'])
    print(accel_data['y'])
    print(accel_data['z'])
    gyro_data = mpu.get_gyro_data()
    print(gyro_data['x'])
    print(gyro_data['y'])
    print(gyro_data['z'])
