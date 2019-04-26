#!/usr/bin/env python3.5

import os
import spidev
import time
import sys
import struct
#import numpy as np
from navio.mpu9250 import MPU9250
from navio.lsm9ds1 import LSM9DS1
from navio import ublox
from navio.ms5611 import MS5611


#UPDATE_RATE = 80.0    # Update Rate in Hz

# Initialize sensors
mpu = MPU9250()
lsm = LSM9DS1()
baro = MS5611()
mpu.initialize()
lsm.initialize()
baro.initialize()

# Test connection:
if mpu.testConnection() and lsm.testConnection():
    print("Connection working.")
else:
    print("Connection to one of the sensors is faulty.")

# GNSS
ubl = ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

ubl.configure_poll_port()
ubl.configure_poll(ublox.CLASS_CFG, ublox.MSG_CFG_USB)
# ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW)

ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
ubl.configure_port(port=ublox.PORT_USB, inMask=1, outMask=1)
ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
ubl.configure_poll_port()
ubl.configure_poll_port(ublox.PORT_SERIAL1)
ubl.configure_poll_port(ublox.PORT_SERIAL2)
ubl.configure_poll_port(ublox.PORT_USB)
ubl.configure_solution_rate(rate_ms=1000)

ubl.set_preferred_dynamic_model(None)
ubl.set_preferred_usePPP(None)

ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSLLH, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_STATUS, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SOL, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELNED, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SVINFO, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELECEF, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSECEF, 1)
ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_RAW, 1)
ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SFRB, 1)
ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SVSI, 1)
ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_ALM, 1)
ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_EPH, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_TIMEGPS, 5)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_CLOCK, 5)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_DGPS, 5)

t_s = time.time()

# find new filename
fileending=1
while os.path.isfile('/home/pi/Navio2/Python/testrun_{}_IMU.txt'.format(fileending)) is True:
    fileending += 1

# Main loop
with open('/home/pi/Navio2/Python/testrun_{}_IMU.txt'.format(fileending), 'w') as dat_imu, \
        open('/home/pi/Navio2/Python/testrun_{}_GNSS.txt'.format(fileending), 'w') as dat_gnss:

    dat_imu.write('t[s], mpu_accel_1, mpu_accel_2, mpu_accel_3, mpu_gyro_1, mpu_gyro_2, mpu_gyro_3, '
                  'mpu_magn_1, mpu_magn_2, mpu_magn_3, '
                  'lsm_accel_1, lsm_accel_2, lsm_accel_3, lsm_gyro_1, lsm_gyro_2, lsm_gyro_3, '
                  'lsm_magn_1, lsm_magn_2, lsm_magn_3, pressure, temp\n')
    dat_gnss.write('t[s], gnss\n')

    t_l = 0.0
    while True:
        t_a = time.time() - t_s

        baro.refreshPressure()
        baro.refreshTemperature()
        mpudata_a, mpudata_g, mpudata_m = mpu.getMotion9()
        lsmdata_a, lsmdata_g, lsmdata_m = lsm.getMotion9()
        baro.readPressure()
        baro.readTemperature()

        baro._calculatePressureAndTemperature()

        # GNSS
        if t_a - t_l > 1.0:
            t_l = t_a
            msg = ubl.receive_message()
            if msg is None:
                if opts.reopen:
                    ubl.close()
                    ubl = ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
                    continue
                print(empty)
                break
            if msg.name() == "NAV_POSLLH":
                outstr = str(msg).split(",")[1:]
                outstr = "".join(outstr)
                dat_gnss.write(str(t_a) + outstr + "\n")
                print(outstr)
            elif msg.name() == "NAV_STATUS":
                outstr = str(msg).split(",")[1:2]
                outstr = "".join(outstr)
                dat_gnss.write(str(t_a) + outstr + "\n")
                print(outstr)


        data = [t_a] + mpudata_a + mpudata_g + mpudata_m + lsmdata_a + lsmdata_g + lsmdata_m + [baro.PRES] + [baro.TEMP]

        # print(data)
        dat_imu.write(str(data) + "\n")
        time.sleep(0.001)
