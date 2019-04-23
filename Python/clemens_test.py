#!/usr/bin/env python

"""This code is written for python2 -.-. Thanks for nothing."""

import spidev
import time
import sys
import navio.mpu9250
from navio.mpu9250 import MPU9250
from navio.lsm9ds1 import LSM9DS1

# Initialize sensors
mpu= MPU9250()
lsm= LSM9DS1()
mpu.initialize()
lsm.initialize()

# Test connection:
if mpu.testConnection() and lsm.testConnection():
    print "Connection working."
else:
    print "Connection to one of the sensors is faulty."

t_s = time.time()

with open('test.txt', 'w') as dat:
    dat.write('t[s], mpu_accel, mpu_gyro, mpu_magn, lsm_accel, lsm_gyro, lsm_magn\n')
    while True:
        t_a = time.time() - t_s
        mpudata_a, mpudata_g, mpudata_m = mpu.getMotion9()
        lsmdata_a, lsmdata_g, lsmdata_m = lsm.getMotion9()
	
	data ="{}, {}, {}, {}, {}, {}, {}\n".format(t_a, mpudata_a, mpudata_g, mpudata_m, lsmdata_a, lsmdata_g, lsmdata_m).replace("[", "").replace("]","")
	print data
        #dat.write(data)
	time.sleep(0.001)

