"""
MS5611 driver code is placed under the BSD license.
Copyright (c) 2014, Emlid Limited, www.emlid.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.
	* Neither the name of the Emlid Limited nor the names of its contributors
	may be used to endorse or promote products derived from this software
	without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import time
from smbus2 import SMBus
import spidev


class MS5611(object):

	class SPIBus(object):
		def __init__(self, bustype=None, spi_bus_number=None, spi_dev_number=None):
			self.bus_type = bustype
			self.bus = spidev.SpiDev()
			self.spi_bus_number = spi_bus_number
			self.spi_dev_number = spi_dev_number

		def open(self):
			self.bus.open(self.spi_bus_number, self.spi_dev_number)
			self.bus.max_speed_hz = 1000000

		def write_register(self, reg_address, data = 0x00):
			self.open()
			tx = [reg_address, data]
			rx = self.bus.xfer2(tx)
			self.bus.close()
			return rx

		def read_registers(self, reg_address, length=3):
			self.open()
			tx = [0] * (length + 1)
			tx[0] = reg_address
			rx = self.bus.xfer2(tx)
			self.bus.close()
			return rx[1:len(rx)]

	class I2CBus(object):
		def __init__(self, I2C_bus_number, address):
			self.bus = SMBus(I2C_bus_number)
			self.address = address

		def write_register(self, reg_address):
			return self.bus.write_byte(self.address, reg_address)

		def read_registers(self, reg_address, length=3):
			data = self.bus.read_i2c_block_data(self.address, reg_address, length)
			print("reg {}: {}".format(reg_address, data))
			return data

	# Minimal and Maximal detectable pressures in mbar and temperatures in °C
	_PRESSURE_MIN = 10.0
	_PRESSURE_MAX = 1200.0
	_TEMP_MIN = -40.0
	_TEMP_MAX = 85.0
	_TEMP_REF = 20.0

	# Registers
	_MS5611_I2C_ADDRESS_CSB_HIGH = 0x76
	_MS5611_I2C_ADDRESS_CSB_LOW  = 0x77
	_MS5611_DEFAULT_ADDRESS      = 0x77  # CSB PIN is active low

	# ADC Read. Returns 24bit
	_MS5611_RA_ADC_READ      = 0x00
	_MS5611_RA_RESET         = 0x1E

	# PROM Read. Returns 16bit
	_MS5611_RA_MANUFACTURER  = 0xA0		# Reserved for Manufacturer
	_MS5611_RA_C1            = 0xA2
	_MS5611_RA_C2            = 0xA4
	_MS5611_RA_C3            = 0xA6
	_MS5611_RA_C4            = 0xA8
	_MS5611_RA_C5            = 0xAA
	_MS5611_RA_C6            = 0xAC
	_MS5611_RA_CRC           = 0xAE		# 4-bit CRC for data validity check

	_MS5611_RA_D1_OSR_256    = 0x40
	_MS5611_RA_D1_OSR_512    = 0x42
	_MS5611_RA_D1_OSR_1024   = 0x44
	_MS5611_RA_D1_OSR_2048   = 0x46
	_MS5611_RA_D1_OSR_4096   = 0x48

	_MS5611_RA_D2_OSR_256    = 0x50
	_MS5611_RA_D2_OSR_512    = 0x52
	_MS5611_RA_D2_OSR_1024   = 0x54
	_MS5611_RA_D2_OSR_2048   = 0x56
	_MS5611_RA_D2_OSR_4096   = 0x58

	def __init__(self, I2C_bus_number=1, address=_MS5611_I2C_ADDRESS_CSB_LOW, SPI_bus_number=0, SPI_dev_number=0, bus="I2C"):

		if bus == "I2C":
			self.bus = self.I2CBus(I2C_bus_number, address)
		elif bus == "SPI":
			self.bus = self.SPIBus(SPI_bus_number, SPI_dev_number)
		else:
			raise ValueError("Choose proper bus type. (I2C or SPI)")

		# Calibration data
		self.C1 = 0		 # Pressure Sensitivity
		self.C2 = 0		 # Pressure Offset
		self.C3 = 0		 # Temperature coefficient of pressure sensitivity
		self.C4 = 0		 # Temperature coefficient of pressure offset
		self.C5 = 0		 # Reference temperature
		self.C6 = 0		 # Temperature coefficient of the temperature

		# Measurement values (digital)
		self.D1 = 0		 # Digital pressure value
		self.D2 = 0		 # Digital temperature value

		self.TEMP = 0.0  # Calculated temperature
		self.PRES = 0.0  # Calculated Pressure

		self._initialize()
		print(self.C1)
		print(self.C2)
		print(self.C3)
		print(self.C4)
		print(self.C5)
		print(self.C6)

	def _initialize(self):
		"""The MS6511 Sensor stores 6 values in the EPROM memory that we need in order to calculate the actual
		temperature and pressure.
		These values are calculated/stored at the factory when the sensor is calibrated.
		I probably could have used the read word function instead of the whole block, but I wanted to keep things
		consistent."""
		C1 = self.bus.read_registers(self._MS5611_RA_C1, length=2)	 # Pressure Sensitivity
		time.sleep(0.05)
		C2 = self.bus.read_registers(self._MS5611_RA_C2, length=2)	 # Pressure Offset
		time.sleep(0.05)
		C3 = self.bus.read_registers(self._MS5611_RA_C3, length=2)	 # Temperature coefficient of pressure sensitivity
		time.sleep(0.05)
		C4 = self.bus.read_registers(self._MS5611_RA_C4, length=2)	 # Temperature coefficient of pressure offset
		time.sleep(0.05)
		C5 = self.bus.read_registers(self._MS5611_RA_C5, length=2)	 # Reference temperature
		time.sleep(0.05)
		C6 = self.bus.read_registers(self._MS5611_RA_C6, length=2)	 # Temperature coefficient of the temperature
		time.sleep(0.05)

		## Again here we are converting the 2 8bit packages into a n integer
		self.C1 = (C1[0] << 8) + C1[1]
		self.C2 = (C2[0] << 8) + C2[1]
		self.C3 = (C3[0] << 8) + C3[1]
		self.C4 = (C4[0] << 8) + C4[1]
		self.C5 = (C5[0] << 8) + C5[1]
		self.C6 = (C6[0] << 8) + C6[1]

		self.update()

	def refreshPressure(self, osr=_MS5611_RA_D1_OSR_4096):
		self.bus.write_register(osr)
		time.sleep(0.01)

	def refreshTemperature(self, osr=_MS5611_RA_D2_OSR_4096):
		self.bus.write_register(osr)
		time.sleep(0.01)

	def readPressure(self):
		D1 = self.bus.read_registers(self._MS5611_RA_ADC_READ, length=3)
		self.D1 = (D1[0] << 16) + (D1[1] << 8) + D1[2]

	def readTemperature(self):
		D2 = self.bus.read_registers(self._MS5611_RA_ADC_READ, length=3)
		self.D2 = (D2[0] << 16) + (D2[1] << 8) + D2[2]

	def _calculatePressureAndTemperature(self):
		# Calculate temperature
		# TODO replace value again with self.D2
		dT = 8569150 - self.C5 * 2.0**8
		TEMP = 2000.0 + dT * self.C6 / 2.0**23

		# Calculate temperature compensated pressure
		OFF = self.C2 * 2.0 ** 16 + (self.C4 * dT) / 2.0**7
		SENS = self.C1 * 2.0 ** 15 + (self.C3 * dT) / 2.0**8

		# Second order temperature compensation
		T2 = 0.0
		OFF2 = 0.0
		SENS2 = 0.0
		if TEMP >= 20.0:
			T2 = 0
			OFF2 = 0
			Sens2 = 0
		elif TEMP < 20.0:
			T2 = dT**2 / 2**31
			OFF2 = 5.0*(TEMP - 2000.0)**2 / 2
			SENS2 = 5.0*(TEMP-2000.0)**2 / 4
			if TEMP < -15.0:
				OFF2 = OFF2 + 7.0*(TEMP + 1500.0)**2
				SENS2 = SENS2 + 11.0*(TEMP + 1500.0)**2 / 2.0

		TEMP = TEMP - T2
		OFF = OFF - OFF2
		SENS = SENS - SENS2

		self.TEMP = TEMP
		self.PRES = (self.D1 * SENS / 2.0**21 - OFF) / 2.0**15

	def returnPressure(self):
		return self.PRES

	def returnTemperature(self):
		return self.TEMP

	def update(self):
		self.refreshPressure()
		time.sleep(0.1)
		self.refreshTemperature()
		time.sleep(0.1) 	# Waiting for pressure data ready
		self.readPressure()
		time.sleep(0.1)
		self.readTemperature()
		time.sleep(0.1)

		self._calculatePressureAndTemperature()

	def test(self):
		self._initialize()
		self.update()
		is_pressure_valid = 1000 <= self.PRES <= 1050
		is_temp_valid = -40 <= self.TEMP <= 80

		return is_pressure_valid and is_temp_valid
