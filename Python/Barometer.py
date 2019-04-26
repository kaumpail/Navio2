"""
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
sudo python Barometer_example.py
"""

import time

import navio.ms5611
import navio.util

navio.util.check_apm()

baro = navio.ms5611.MS5611()

while(True):
	baro.update()

	print("Temperature(C): {}".format(baro.TEMP), "Pressure(millibar): {}".format(baro.PRES))

	time.sleep(0.1)
