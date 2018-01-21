"""This program handles the communication over I2C
between a BeagleBone Black and a MPU-6050 Gyroscope / Accelerometer combination
Made by: Sefa Yerli DEneme deneme
"""

import time
import smbus
import math
import numpy
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import collections
import random
import multiprocessing
import pylab
from Adafruit_I2C import Adafruit_I2C
from time import sleep
import Adafruit_BBIO.PWM as PWM
from bbio import *
from bbio.libraries.RotaryEncoder import RotaryEncoder

elapsed = 0.004
start_time = time.time()
address = 0x68
last_x = 0


class MPU6050:
    # Global Variables
    GRAVITIY_MS2 = 9.80665
    # address = 0x68
    bus = smbus.SMBus(1)
    # last_x = 0
    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    ACCEL_CONFIG = 0x1C

    def __init__(self, sampleinterval=0.1, timewindow=100., size=(600, 350)):
        self.address = address

        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

        # Data stuff
        self._interval = int(sampleinterval * 1000)
        self._bufsize = int(timewindow / sampleinterval)
        self.databuffer = collections.deque([0.0] * self._bufsize, self._bufsize)
        self.x = np.linspace(-timewindow, 0.0, self._bufsize)
        self.y = np.zeros(self._bufsize, dtype=np.float)
        # PyQtGraph stuff
        self.app = QtGui.QApplication([])
        self.plt = pg.plot(title='Position and Setpoint Live Graph')
        self.plt.resize(*size)
        self.plt.setRange(yRange=[-10, 10])
        self.plt.showGrid(x=True, y=True)
        self.plt.setLabel('left', 'angle', 'degree')
        self.plt.setLabel('bottom', 'time', 's')
        self.curve = self.plt.plot(self.x, self.y, pen=(255, 0, 0))
        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.start(self._interval)

    def updateplot(self):
        # for x in range(50):
        self.databuffer.append(self.get_accel_data())
        self.y[:] = self.databuffer
        self.curve.setData(self.x, self.y)
        self.app.processEvents()

    def run(self):
        self.app.exec_()

    # I2C communication methods
    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.
        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    # MPU-6050 Methods

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.
        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

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

    def read_accel_range(self, raw=False):
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

    def get_accel_data(self, g=False):
        """Gets and returns the X, Y and Z values from the accelerometer.
        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        global last_x

        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = self.set_accel_range(self.ACCEL_RANGE_16G)
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
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier
        # print "Ax	 reci_x		degree		sin(degree)"
        print"X		Y      	Z	radian_x	cos 	sin  	x_1"
        if g is True:
            print("g is true")
            return int(round(x1, 1))

        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            radians_x = math.degrees(math.atan2(x, math.sqrt((y * y)) + (z * z)))

            x1 = x - (math.sin(math.radians(radians_x)) * 9.780)
            # last_x = last_x * (1-1/(2*2)) + x1 * (1/(2*2))
            print(round(x, 3)), "      ",
            """print(last_x), "    ",
                print(round(x1, 1)), "   ",
                print(round(radians_x,2)), "  ",
                print(round(math.sin(radians_x),2))"""
            print(round(y, 3)), " ",
            print(round(z, 3)), "	",
            print(round(radians_x, 2)), "	",
            # z = z - (math.cos(radians_x) * 9.780)
            print(round(math.cos(math.radians((radians_x))), 2)), "	",
            print(round(math.sin(math.radians((radians_x))), 2)), "	",
            # deneme = z + x - self.GRAVITIY_MS2
            print(x1)
            return round(x1, 1)

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()

        return [accel, temp]


if __name__ == "__main__":
    t = time.time()
    mpu = MPU6050(sampleinterval=0.05, timewindow=10.)
    mpu.run()