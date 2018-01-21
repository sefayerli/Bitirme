###################################################################################
# realtimesliding.py         					                  #
# Author : eyazici							 	  #
# -This code is controlling the test bench by using the sliding mode controller.  #
# -The real time plotting is avaliable in the code.     		 	  #
# -The position angle is obtained by using encoder.			 	  #
# -The setpoint is updated according the turn variable and code can be updated    #
#  to glove use							           	  #
# -To finish the infine loop close the real time plotting graph.         	  #
###################################################################################
#dasdfwşwkelnf
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import collections
import random
import time
import math
import numpy as np
import multiprocessing
import random
import numpy
import pylab
from Adafruit_I2C import Adafruit_I2C
from time import sleep
import math
import time
import Adafruit_BBIO.PWM as PWM
from bbio import *
from bbio.libraries.RotaryEncoder import RotaryEncoder

global setpoint, error, lasterror, errsum, output, Kp, Kd, Ki, pid, i

setpoint = 60
error = 0
lasterror = 0
errsum = 0
output = 5800
i = 0
elapsed = 0.004
encoder = RotaryEncoder(RotaryEncoder.EQEP2b)
encoder.setAbsolute()
encoder.zero()

start_time=time.time()
# initialize i2c connection to MPU6050
# i2c address is 0x68
i2c = Adafruit_I2C(0x68)

global gyro_x_angle
global last_x_angle

gyro_x_angle = 0
last_x_angle = 0

class DynamicPlotter():
    def __init__(self, sampleinterval=0.1, timewindow=1000., size=(600,350)):
        # Data stuff
        self._interval = int(sampleinterval*1000)
        self._bufsize = int(timewindow/sampleinterval)
        self.databuffer = collections.deque([0.0]*self._bufsize, self._bufsize)
        self.x = np.linspace(-timewindow, 0.0, self._bufsize)
        self.y = np.zeros(self._bufsize, dtype=np.float)
        # PyQtGraph stuff
        self.app = QtGui.QApplication([])
        self.plt = pg.plot(title='Position and Setpoint Live Graph')
        self.plt.resize(*size)
        self.plt.setRange(yRange=[0,100])
        self.plt.showGrid(x=True, y=True)
        self.plt.setLabel('left', 'angle', 'degree')
        self.plt.setLabel('bottom', 'time', 's')
        self.curve = self.plt.plot(self.x, self.y, pen=(255,0,0))
        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.start(self._interval)

    def getdata(self):
        t = time.time()
        global elapsed, gyro_x_angle, last_x_angle
        global setpoint, error, lasterror, errsum, output, Kp, Kd, Ki, pid, i
        ######################### GYROSCOPE SENSOR DATA READING #########################
        #Reading x-gyro data
        xhigh = i2c.readS8(0x43)
        xlow = i2c.readU8(0x44)
        #converting 8-bits data to 16-bits
        rawx= xhigh*256 + xlow
        #reducing the calculated offset of the gyro sensor
        gyro_xout = rawx
        #dividing read data with the given rate of gyro
        gyro_x_rate = gyro_xout/131.
        ################################################################################
        ####################### ACCELEROMETER SENSOR DATA READING ######################
        #obtaining g-force of the y-axis
        by= i2c.readS8(0x3d)
        sy= i2c.readU8(0x3e)
        rawy= by * 256 + sy
        gy= rawy / 16384.
        #obtaining g-force of the z-axis
        bz= i2c.readS8(0x3f)
        sz= i2c.readU8(0x40)
        rawz= bz * 256 + sz
        gz= rawz /16384.
        #calculating euler angles by using arctan
        roll=(math.atan2(rawy,rawz)*57.2957786)
        ################################################################################
        ####################### COMPLEMENTARY FILTER OF DATA############################
        last_x_angle = (0.966)* (last_x_angle + gyro_x_rate * 0.0262) + (0.034)*(roll)
        ################################################################################
	    angle_encoder = float(encoder.getPosition())*0.18 + 55
        angle_encoder1 = int(angle_encoder)
        setpoint = last_x_angle + 90
        error = setpoint - angle_encoder
        derror = (error - lasterror)/elapsed
        errsum += error*elapsed
        sliding_surface = derror + 5*error


        if setpoint-1<angle_encoder1<setpoint+1:
                if sliding_surface>0:
                        out_sliding = 0.05
                else:
                        out_sliding = -0.05
        elif setpoint-3<angle_encoder1<setpoint+3:
                if sliding_surface>0:
                        out_sliding = 0.1
                else:
                        out_sliding = -0.1
        elif setpoint-5<angle_encoder1<setpoint+5:
                if sliding_surface>0:
                        out_sliding = 0.2
                else:
                        out_sliding = -0.2
        elif setpoint-7<angle_encoder1<setpoint+7:
                if sliding_surface>0:
                        out_sliding = 0.4
                else:
                        out_sliding = -0.4
        elif setpoint-9<angle_encoder1<setpoint+9:
                if sliding_surface>0:
                        out_sliding = 1
                else:
                        out_sliding = -1
        else:
                if sliding_surface>0:
                        out_sliding = 2
                else:
                        out_sliding = -2
        output = float(output) + out_sliding
        print("Sample Time = " + str(elapsed) +" and encoder :  " + str(angle_encoder1) + " setpoint : " + str(setpoint))
        lasterror = error
        if 5400 <= output < 6200:
                PWM.set_duty_cycle("P8_13", output/1000)
        else:
                output=output
        i = i + 1
        if i == 1000:
                setpoint = 70
        elapsed = time.time() - t
        return angle_encoder

    def updateplot(self):
        for x in range(50):
                self.databuffer.append( self.getdata() )
        self.y[:] = self.databuffer
        self.curve.setData(self.x, self.y)
        self.app.processEvents()

    def run(self):
        self.app.exec_()

if __name__ == '__main__':
        print("PWM starting...")
        PWM.start("P8_13",4, 50.0)
        sleep(10)
        print("PWM started, duty cycle reaching 4...")
        PWM.set_duty_cycle("P8_13", 4)
        sleep(3)
        print("PWM is 4, reaching 4.4 movement is starting...")
        PWM.set_duty_cycle("P8_13", 4.4)
        sleep(3)
        print("PWM is 4.4, reaching 5.4 movement is starting...")
        PWM.set_duty_cycle("P8_13", 5.4)
        sleep(2)
        print("Sliding Control is starting..")
        sleep(2)
        m = DynamicPlotter(sampleinterval=0.05, timewindow=100.)
        m.run()
        print("stopped")
        PWM.set_duty_cycle("P8_13", 5.7)
        sleep(0.5)
        PWM.set_duty_cycle("P8_13", 5.6)
        sleep(0.5)
        PWM.set_duty_cycle("P8_13", 5.5)
        sleep(0.5)
        PWM.set_duty_cycle("P8_13", 5)
        sleep(0.5)
        PWM.set_duty_cycle("P8_13", 4)
        sleep(0.5)
        PWM.stop("P8_13")
        PWM.cleanup()


