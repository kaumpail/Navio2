#!/usr/bin/env python3

import os
import time
import sys
import struct
import csv
#import numpy as np
from navio.mpu9250 import MPU9250
from navio.lsm9ds1 import LSM9DS1
from navio import ublox
from navio.ms5611 import MS5611
from navio.leds import Led

def main():
    # Initialize sensors
    #   Initialize IMUs
    mpu = MPU9250()
    mpu.initialize(low_pass_filter=1)
    mpu.set_acc_scale(0x08)       # +/-4G
    mpu.set_gyro_scale(0x08)      # +/-500dps

    lsm = LSM9DS1()
    lsm.initialize()
    lsm.set_acc_scale(0x10)     # +/-4G
    lsm.set_gyro_scale(0x00)    # +/-245dps
    lsm.set_mag_scale(0x20)     # +/-8Gs

    #     Test connection:
    if mpu.testConnection() and lsm.testConnection():
        print("Connection working.")
    else:
        print("Connection to one of the sensors is faulty.")

    #   Initialize barometer
    baro = MS5611()

    #   initialize UBlox
    ubl = ublox.UBlox("spi:0.0", baudrate=5000000)

    #     reset everything
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

    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSLLH, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELNED, 1)

    time.sleep(0.1)

    t_s = time.time()

    # find new filename
    fileending = 1
    while os.path.isfile('meas_data/datafile_{}_IMU.txt'.format(fileending)) is True:
        fileending += 1

    # open('', '', 1) enables line buffering
    with open('meas_data/datafile_{}_IMU.txt'.format(fileending), 'w', 1) as dat_imu, \
            open('meas_data/datafile_{}_GNSS_pos_lla.txt'.format(fileending), 'w', 1) as dat_gnss_pos_lla, \
            open('meas_data/datafile_{}_GNSS_vel_ned.txt'.format(fileending), 'w', 1) as dat_gnss_vel_ned, \
            open('meas_data/datafile_{}_baro.txt'.format(fileending), 'w', 1) as dat_baro:

        # write headers to file
        dat_imu.write('t[s], mpu_accel_1, mpu_accel_2, mpu_accel_3, mpu_gyro_1, mpu_gyro_2, mpu_gyro_3, '
                      'mpu_magn_1, mpu_magn_2, mpu_magn_3, '
                      'lsm_accel_1 [m/s], lsm_accel_2 [m/s], lsm_accel_3 [m/s], '
                      'lsm_gyro_1 [rad/s], lsm_gyro_2 [rad/s], lsm_gyro_3 [rad/s], '
                      'lsm_magn_1 [gauss], lsm_magn_2 [gauss], lsm_magn_3 [gauss]\n')
        dat_gnss_pos_lla.write('t[s], iTOW [ms], lon [1e-7 lon], lat [1e-7 deg], height [mm], height above mean see level [mm], '
                               'Horizontal accuracy estimate [mm], Vertical accuracy estimate [mm]\n')
        dat_gnss_vel_ned.write('t[s], iTOW [ms], velN [cm/s], velE [cm/s], velD [cm/s], speed [cm/s], '
                               'groundspeed [cm/s], heading [1e-5 deg], sAcc [cm/s], cAcc [1e-5 deg]\n')
        dat_baro.write('t[s], pressure [mbar], temperature [°C]\n')

        # Main loop
        t_l = 0.0  # time of last gnss & baro measurement
        while True:
            t_a = time.time() - t_s

            mpudata_a, mpudata_g, mpudata_m = mpu.getMotion9()
            lsmdata_a, lsmdata_g, lsmdata_m = lsm.getMotion9()

            # GNSS & barometer
            if t_a - t_l > 0.5:
                t_l = t_a
                baro.update()
                msg = ubl.receive_message()

                if msg.name() == "NAV_POSLLH":
                    dat_gnss_pos_lla.write("{}, {}\n".format(t_a, str(struct.unpack('<IiiiiII', msg._buf[6:34])).replace("(", "").replace(")", "")))
                    # dat_gnss.flush()
                elif msg.name() == "NAV_VELNED":
                    dat_gnss_vel_ned.write("{}, {}\n".format(t_a, str(struct.unpack('<IiiiIIiII', msg._buf[6:42])).replace("(", "").replace(")", "")))

                dat_baro.write("{}, {}, {}\n".format(t_a, baro.returnPressure(), baro.returnTemperature()))
                # dat_baro.flush()

            data = [t_a] + mpudata_a + mpudata_g + mpudata_m + lsmdata_a + lsmdata_g + lsmdata_m

            # print(data)
            dat_imu.write(str(data).replace("[", "").replace("]", "") + "\n")
            # dat_imu.flush()

            time.sleep(0.005)  # sleep time to restrict update rate


if __name__ == "__main__":

    led = Led()
    led.setColor('Green')

    errfile = "errorfile.txt"

    with open(errfile, 'w', 1) as efile:
        try:
            main()
        except KeyboardInterrupt:
            led.setColor('Yellow')
        except Exception as e:
            led.setColor('Red')
            efile.write(e)
