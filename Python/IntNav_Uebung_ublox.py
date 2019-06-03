#!/usr/bin/env python3

import time
from navio import ublox
import os


def main():

    # initialize UBlox
    ubl = ublox.UBlox("/dev/ttyACM1", baudrate=57600)

    # ubl.set_debug(1)
    ubl.set_binary()
    #  reset everything
    ubl.configure_loadsave(clearMask=0b1111100011111, deviceMask=0b10110)

    ubl.set_preferred_dynamic_model(None)
    ubl.set_preferred_usePPP(None)

    ubl.configure_message_rate(ublox.CLASS_ESF, ublox.MSG_ESF_RAW, 1)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_RAWX, 1)

    # find new filename
    fileending = 1
    while True:
        if os.path.isfile('/home/pi/Navio2/Python/meas_data/datafile_{}_IMU.txt'.format(fileending)) is True:
            fileending += 1

    time.sleep(0.1)
    with open('/home/pi/Navio2/Python/meas_data/datafile_{}_ESF-RAW.txt'.format(fileending), 'w', 1) as dat_ESF, \
            open('/home/pi/Navio2/Python/meas_data/datafile_{}_RXM-RAW.txt'.format(fileending), 'w', 1) as dat_RXM:

        while True:
            msg = ubl.receive_message()
            if msg is None:
                continue
            elif msg.name() == "ESF_RAW":
                dat_ESF.write(msg.__str__() + "\n")
            elif msg.name() == "RXM_RAWX":
                dat_RXM.write(msg.__str__() + "\n")

            time.sleep(0.2)

    # with open("/dev/ttyACM1", 'rb') as ufile:
    #     while True:
    #         print(ufile.readline())
    #         time.sleep(0.1)


if __name__ == "__main__":
    main()
