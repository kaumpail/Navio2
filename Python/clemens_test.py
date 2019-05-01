#!/usr/bin/env python3.5

import os
import spidev
import time
import sys
import struct
import csv
#import numpy as np
from navio.mpu9250 import MPU9250
from navio.lsm9ds1 import LSM9DS1
from navio import ublox
from navio.ms5611 import MS5611


# Initialize sensors
mpu = MPU9250()
mpu.initialize()

lsm = LSM9DS1()
lsm.initialize()

baro = MS5611()

# Test connection:
if mpu.testConnection() and lsm.testConnection():
    print("Connection working.")
else:
    print("Connection to one of the sensors is faulty.")

# GNSS
ubl = ublox.UBlox("spi:0.0", baudrate=5000000)

# reset everything
ubl.configure_loadsave(clearMask=0b1111100011111, deviceMask=0b10111)

#ubl.configure_poll_port()
#ubl.configure_poll(ublox.CLASS_CFG, ublox.MSG_CFG_USB)
# ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW)

#ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
#ubl.configure_port(port=ublox.PORT_USB, inMask=1, outMask=1)
#ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
#ubl.configure_poll_port()
#ubl.configure_poll_port(ublox.PORT_SERIAL1)
#ubl.configure_poll_port(ublox.PORT_SERIAL2)
#ubl.configure_poll_port(ublox.PORT_USB)
ubl.configure_poll_port(ublox.PORT_SPI)
ubl.configure_solution_rate(rate_ms=200, nav_rate=5)

ubl.set_preferred_dynamic_model(None)
ubl.set_preferred_usePPP(None)

# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSLLH, 1)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_STATUS, 1)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SOL, 1)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELNED, 1)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SVINFO, 1)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELECEF, 1)
ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSECEF, 1)
# ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_RAW, 1)
# ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SFRB, 1)
# ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SVSI, 1)
# ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_ALM, 1)
# ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_EPH, 1)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_TIMEGPS, 5)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_CLOCK, 5)
# ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_DGPS, 1)

t_s = time.time()

# find new filename
fileending=1
while os.path.isfile('/home/pi/Navio2/Python/testrun_{}_IMU.txt'.format(fileending)) is True:
    fileending += 1

# write headers to file
with open('/home/pi/Navio2/Python/testrun_{}_IMU.txt'.format(fileending), 'w') as dat_imu, \
        open('/home/pi/Navio2/Python/testrun_{}_GNSS.txt'.format(fileending), 'w') as dat_gnss, \
        open('/home/pi/Navio2/Python/testrun_{}_baro.txt'.format(fileending), 'w') as dat_baro:

    dat_imu.write('t[s], mpu_accel_1, mpu_accel_2, mpu_accel_3, mpu_gyro_1, mpu_gyro_2, mpu_gyro_3, '
                  'mpu_magn_1, mpu_magn_2, mpu_magn_3, '
                  'lsm_accel_1, lsm_accel_2, lsm_accel_3, lsm_gyro_1, lsm_gyro_2, lsm_gyro_3, '
                  'lsm_magn_1, lsm_magn_2, lsm_magn_3\n')
    dat_gnss.write('t[s], iTOW, ecefX [cm], ecefY [cm], ecefZ [cm], pAcc\n')
    dat_baro.write('t[s], pressure [mbar], temperature [°C]\n')


# Main loop
# As Power loss seems to corrupt the files the files are appended in batches
t_l1 = 0.0  # time of last gnss & baro measurement
t_l2 = 0.0 # time of last write to file
while True:
    t_a = time.time() - t_s
    imudata = []
    gnssdata = []
    barodata = []

    mpudata_a, mpudata_g, mpudata_m = mpu.getMotion9()
    lsmdata_a, lsmdata_g, lsmdata_m = lsm.getMotion9()

    # GNSS & barometer
    if t_a - t_l1 > 1.0:
        t_l1 = t_a
        baro.update()
        msg = ubl.receive_message()
        if msg is None:
            if opts.reopen:
                ubl.close()
                ubl = ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
                continue
            print(empty)
            break
        if msg.name() == "NAV_POSECEF":
            gnssdata.append("{}, {}".format(t_a, str(struct.unpack('<IiiiI', msg._buf[6:26])).replace("(", "").replace(")", "")))

        barodata.append("{}, {}, {}".format(t_a, baro.returnPressure(), baro.returnTemperature()))


    data = [t_a] + mpudata_a + mpudata_g + mpudata_m + lsmdata_a + lsmdata_g + lsmdata_m

    # print(data)
    imudata.append(str(data).replace("[", "").replace("]", ""))

    # Write data to file every 10 seconds
    if t_a - t_l2 > 10.0:
        t_l2 = t_a
        with open('/home/pi/Navio2/Python/testrun_{}_IMU.txt'.format(fileending), 'a') as dat_imu, \
                open('/home/pi/Navio2/Python/testrun_{}_GNSS.txt'.format(fileending), 'a') as dat_gnss, \
                open('/home/pi/Navio2/Python/testrun_{}_baro.txt'.format(fileending), 'a') as dat_baro:
            for l in imudata:
                dat_imu.write(l + "\n")
            for l in gnssdata:
                dat_gnss.write(l + "\n")
            for l in barodata:
                dat_baro.write(l + "\n")
        imudata = []
        gnssdata = []
        barodata = []

    else:
        time.sleep(0.005) # sleep time to restrict update rate
