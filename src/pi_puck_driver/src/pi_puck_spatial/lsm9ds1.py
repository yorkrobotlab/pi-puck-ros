#!/usr/bin/env python
"""Sensor driver for the LSM9DS1 IMU.

Based on https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1.
"""

import struct
import time

from smbus import SMBus

_LSM9DS1_ADDRESS_ACCELGYRO = 0x6B
_LSM9DS1_ADDRESS_MAG = 0x1E
_LSM9DS1_XG_ID = 0b01101000
_LSM9DS1_MAG_ID = 0b00111101
_LSM9DS1_ACCEL_MG_LSB_2G = 0.061
_LSM9DS1_ACCEL_MG_LSB_4G = 0.122
_LSM9DS1_ACCEL_MG_LSB_8G = 0.244
_LSM9DS1_ACCEL_MG_LSB_16G = 0.732
_LSM9DS1_MAG_MGAUSS_4GAUSS = 0.14
_LSM9DS1_MAG_MGAUSS_8GAUSS = 0.29
_LSM9DS1_MAG_MGAUSS_12GAUSS = 0.43
_LSM9DS1_MAG_MGAUSS_16GAUSS = 0.58
_LSM9DS1_GYRO_DPS_DIGIT_245DPS = 0.00875
_LSM9DS1_GYRO_DPS_DIGIT_500DPS = 0.01750
_LSM9DS1_GYRO_DPS_DIGIT_2000DPS = 0.07000
_LSM9DS1_TEMP_LSB_DEGREE_CELSIUS = 8  # 1 degrees C = 8, 25 degrees = 200, etc.
_LSM9DS1_REGISTER_WHO_AM_I_XG = 0x0F
_LSM9DS1_REGISTER_CTRL_REG1_G = 0x10
_LSM9DS1_REGISTER_CTRL_REG2_G = 0x11
_LSM9DS1_REGISTER_CTRL_REG3_G = 0x12
_LSM9DS1_REGISTER_TEMP_OUT_L = 0x15
_LSM9DS1_REGISTER_TEMP_OUT_H = 0x16
_LSM9DS1_REGISTER_STATUS_REG = 0x17
_LSM9DS1_REGISTER_OUT_X_L_G = 0x18
_LSM9DS1_REGISTER_OUT_X_H_G = 0x19
_LSM9DS1_REGISTER_OUT_Y_L_G = 0x1A
_LSM9DS1_REGISTER_OUT_Y_H_G = 0x1B
_LSM9DS1_REGISTER_OUT_Z_L_G = 0x1C
_LSM9DS1_REGISTER_OUT_Z_H_G = 0x1D
_LSM9DS1_REGISTER_CTRL_REG4 = 0x1E
_LSM9DS1_REGISTER_CTRL_REG5_XL = 0x1F
_LSM9DS1_REGISTER_CTRL_REG6_XL = 0x20
_LSM9DS1_REGISTER_CTRL_REG7_XL = 0x21
_LSM9DS1_REGISTER_CTRL_REG8 = 0x22
_LSM9DS1_REGISTER_CTRL_REG9 = 0x23
_LSM9DS1_REGISTER_CTRL_REG10 = 0x24
_LSM9DS1_REGISTER_OUT_X_L_XL = 0x28
_LSM9DS1_REGISTER_OUT_X_H_XL = 0x29
_LSM9DS1_REGISTER_OUT_Y_L_XL = 0x2A
_LSM9DS1_REGISTER_OUT_Y_H_XL = 0x2B
_LSM9DS1_REGISTER_OUT_Z_L_XL = 0x2C
_LSM9DS1_REGISTER_OUT_Z_H_XL = 0x2D
_LSM9DS1_REGISTER_WHO_AM_I_M = 0x0F
_LSM9DS1_REGISTER_CTRL_REG1_M = 0x20
_LSM9DS1_REGISTER_CTRL_REG2_M = 0x21
_LSM9DS1_REGISTER_CTRL_REG3_M = 0x22
_LSM9DS1_REGISTER_CTRL_REG4_M = 0x23
_LSM9DS1_REGISTER_CTRL_REG5_M = 0x24
_LSM9DS1_REGISTER_STATUS_REG_M = 0x27
_LSM9DS1_REGISTER_OUT_X_L_M = 0x28
_LSM9DS1_REGISTER_OUT_X_H_M = 0x29
_LSM9DS1_REGISTER_OUT_Y_L_M = 0x2A
_LSM9DS1_REGISTER_OUT_Y_H_M = 0x2B
_LSM9DS1_REGISTER_OUT_Z_L_M = 0x2C
_LSM9DS1_REGISTER_OUT_Z_H_M = 0x2D
_LSM9DS1_REGISTER_CFG_M = 0x30
_LSM9DS1_REGISTER_INT_SRC_M = 0x31
_MAGTYPE = True
_XGTYPE = False
_SENSORS_GRAVITY_STANDARD = 9.80665

ACCELRANGE_2G = (0b00 << 3)
ACCELRANGE_16G = (0b01 << 3)
ACCELRANGE_4G = (0b10 << 3)
ACCELRANGE_8G = (0b11 << 3)
MAGGAIN_4GAUSS = (0b00 << 5)  # +/- 4 gauss
MAGGAIN_8GAUSS = (0b01 << 5)  # +/- 8 gauss
MAGGAIN_12GAUSS = (0b10 << 5)  # +/- 12 gauss
MAGGAIN_16GAUSS = (0b11 << 5)  # +/- 16 gauss
GYROSCALE_245DPS = (0b00 << 3)  # +/- 245 degrees/s rotation
GYROSCALE_500DPS = (0b01 << 3)  # +/- 500 degrees/s rotation
GYROSCALE_2000DPS = (0b11 << 3)  # +/- 2000 degrees/s rotation


class LSM9DS1:
    """Driver for the LSM9DS1 accelerometer, magnetometer, gyroscope."""

    def __init__(self, i2c_bus=3, mag_address=0x1e, xg_address=0x6b):
        """Initialise the LSM9DS1."""
        self._bus = SMBus(i2c_bus)
        self._mag_address = mag_address
        self._xg_address = xg_address
        self._buffer = bytearray(6)

        # soft reset & reboot accel/gyro
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG8, 0x05)
        # soft reset & reboot magnetometer
        self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C)
        time.sleep(0.01)
        # Check ID registers.
        if self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_XG) != _LSM9DS1_XG_ID or \
           self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_M) != _LSM9DS1_MAG_ID:
            raise RuntimeError('Could not find LSM9DS1, check wiring!')
        # enable gyro continuous
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0) # on XYZ
        # Enable the accelerometer continous
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38)
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0)
        # enable mag continuous
        self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG3_M, 0x00)
        # Set default ranges for the various sensors
        self._accel_mg_lsb = None
        self._mag_mgauss_lsb = None
        self._gyro_dps_digit = None
        self.accel_range = ACCELRANGE_2G
        self.mag_gain = MAGGAIN_4GAUSS
        self.gyro_scale = GYROSCALE_245DPS

    @property
    def accel_range(self):
        """Accelerometer range.

        Must be a value of:
          - ACCELRANGE_2G
          - ACCELRANGE_4G
          - ACCELRANGE_8G
          - ACCELRANGE_16G
        """
        reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL)
        return (reg & 0b00011000) & 0xFF

    @accel_range.setter
    def accel_range(self, val):
        assert val in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G,
                       ACCELRANGE_16G)
        reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, reg)
        if val == ACCELRANGE_2G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_2G
        elif val == ACCELRANGE_4G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_4G
        elif val == ACCELRANGE_8G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_8G
        elif val == ACCELRANGE_16G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_16G

    @property
    def mag_gain(self):
        """Magnetometer gain.

        Must be a value of:
          - MAGGAIN_4GAUSS
          - MAGGAIN_8GAUSS
          - MAGGAIN_12GAUSS
          - MAGGAIN_16GAUSS
        """
        reg = self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
        return (reg & 0b01100000) & 0xFF

    @mag_gain.setter
    def mag_gain(self, val):
        assert val in (MAGGAIN_4GAUSS, MAGGAIN_8GAUSS, MAGGAIN_12GAUSS,
                       MAGGAIN_16GAUSS)
        reg = self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
        reg = (reg & ~(0b01100000)) & 0xFF
        reg |= val
        self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, reg)
        if val == MAGGAIN_4GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_4GAUSS
        elif val == MAGGAIN_8GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_8GAUSS
        elif val == MAGGAIN_12GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_12GAUSS
        elif val == MAGGAIN_16GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_16GAUSS

    @property
    def gyro_scale(self):
        """Gyroscope scale.

        Must be a value of:
          - GYROSCALE_245DPS
          - GYROSCALE_500DPS
          - GYROSCALE_2000DPS
        """
        reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G)
        return (reg & 0b00011000) & 0xFF

    @gyro_scale.setter
    def gyro_scale(self, val):
        assert val in (GYROSCALE_245DPS, GYROSCALE_500DPS, GYROSCALE_2000DPS)
        reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, reg)
        if val == GYROSCALE_245DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_245DPS
        elif val == GYROSCALE_500DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_500DPS
        elif val == GYROSCALE_2000DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_2000DPS

    def read_accel_raw(self):
        """Read raw accelerometer sensor and return it as a 3-tuple of X, Y, Z values that are 16-bit unsigned values.

        If you want the acceleration in nice units you probably want to use the accelerometer property!
        """
        # Read the accelerometer
        self._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_XL, 6,
                         self._buffer)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._buffer[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def acceleration(self):
        """Accelerometer X, Y, Z axis values as a 3-tuple of m/s^2 values."""
        raw = self.read_accel_raw()
        return map(lambda x: x * self._accel_mg_lsb / 1000.0 * _SENSORS_GRAVITY_STANDARD,
                   raw)

    def read_mag_raw(self):
        """Read raw magnetometer sensor and return as a 3-tuple of X, Y, Z values that are 16-bit unsigned values.

        If you want the magnetometer in nice units you probably want to use the magnetometer property!
        """
        # Read the magnetometer
        self._read_bytes(_MAGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_M, 6,
                         self._buffer)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._buffer[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def magnetic(self):
        """Magnetometer X, Y, Z axis values as a 3-tuple of gauss values."""
        raw = self.read_mag_raw()
        return map(lambda x: x * self._mag_mgauss_lsb / 1000.0, raw)

    def read_gyro_raw(self):
        """Read raw gyroscope sensor values and return as a 3-tuple of X, Y, Z values that are 16-bit unsigned values.

        If you want the gyroscope in nice units you probably want to use the gyroscope property!
        """
        # Read the gyroscope
        self._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_G, 6,
                         self._buffer)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._buffer[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def gyro(self):
        """Gyroscope X, Y, Z axis values as a 3-tuple of degrees/second values."""
        raw = self.read_gyro_raw()
        return map(lambda x: x * self._gyro_dps_digit, raw)

    def read_temp_raw(self):
        """Read the raw temperature sensor value and return it as a 12-bit signed value.

        If you want the temperature in nice units you probably want to use the temperature property!
        """
        # Read temp sensor
        self._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_TEMP_OUT_L, 2,
                         self._buffer)
        temp = ((self._buffer[1] << 8) | self._buffer[0]) >> 4
        return _twos_comp(temp, 12)

    @property
    def temperature(self):
        """Temperature of the sensor in degrees Celsius."""
        # This is just a guess since the starting point (21C here) isn't documented :(
        # See discussion from:
        #  https://github.com/kriswiner/LSM9DS1/issues/3
        temp = self.read_temp_raw()
        temp = 27.5 + temp/16
        return temp

    def _read_u8(self, sensor_type, address):
        if sensor_type == _MAGTYPE:
            i2c_address = self._mag_address
        else:
            i2c_address = self._xg_address

        self._buffer[0] = address & 0xFF
        self._buffer[1] = self._bus.read_byte_data(i2c_address, self._buffer[0])
        return self._buffer[1]

    def _read_bytes(self, sensor_type, address, count, buf):
        if sensor_type == _MAGTYPE:
            i2c_address = self._mag_address
        else:
            i2c_address = self._xg_address

        self._buffer[0] = address & 0xFF
        for index, item in enumerate(self._bus.read_i2c_block_data(i2c_address, self._buffer[0], count)):
            self._buffer[index] = item

    def _write_u8(self, sensor_type, address, val):
        if sensor_type == _MAGTYPE:
            i2c_address = self._mag_address
        else:
            i2c_address = self._xg_address

        self._buffer[0] = address & 0xFF
        self._buffer[1] = val & 0xFF
        self._bus.write_byte_data(i2c_address, self._buffer[0], self._buffer[1])

    def close(self):
        """Close the I2C bus."""
        self._bus.close()
