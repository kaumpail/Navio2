#!/usr/bin/env python3

import time
from navio import ublox


def main():

    # initialize UBlox
    ubl = ublox.UBlox("/dev/ttyACM1")

    #  reset everything
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
    ##ubl.configure_poll_port(ublox.PORT_SPI)
    ##ubl.configure_solution_rate(rate_ms=200, nav_rate=5)

    #ubl.set_preferred_dynamic_model(None)
    #ubl.set_preferred_usePPP(None)

    #ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSLLH, 1)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_STATUS, 1)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SOL, 1)
    #ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELNED, 1)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SVINFO, 1)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELECEF, 1)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSECEF, 1)
    # ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_RAW, 1)
    # ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SFRB, 1)
    # ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SVSI, 1)
    # ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_ALM, 1)
    # ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_EPH, 1)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_TIMEGPS, 5)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_CLOCK, 5)
    # ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_DGPS, 1)

    time.sleep(0.1)

    print(ubl.read_only)


if __name__ == "__main__":
    main()
