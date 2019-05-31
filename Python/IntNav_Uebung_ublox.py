#!/usr/bin/env python3

import time
from navio import ublox
import os


def main():

    # initialize UBlox
    ubl = ublox.UBlox("/dev/ttyACM0", baudrate=57600)

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
    while os.path.isfile('/home/pi/Navio2/Python/datafile_{}_ESF-RAW.txt'.format(fileending)) is True:
        fileending += 1

    time.sleep(0.1)
    with open('/home/pi/Navio2/Python/datafile_{}_ESF-RAW.txt'.format(fileending), 'w', 1) as dat_ESF, \
            open('/home/pi/Navio2/Python/datafile_{}_RXM-RAW.txt'.format(fileending), 'w', 1) as dat_RXM:

        while True:
            msg = ubl.receive_message()
            if type(msg) is None:
                continue
            elif msg.name() == "ESF-RAW":
                dat_ESF.write(msg)
            elif msg.name() == "dat_RXM":
                dat_RXM.write(msg)

            time.sleep(0.2)

    # with open("/dev/ttyACM1", 'rb') as ufile:
    #     while True:
    #         print(ufile.readline())
    #         time.sleep(0.1)


if __name__ == "__main__":
    main()
