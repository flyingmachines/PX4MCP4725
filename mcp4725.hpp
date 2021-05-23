/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file md25.cpp
 *
 * Driver for MD25 I2C Motor Driver
 *
 * references:
 * http://www.robot-electronics.co.uk/htm/md25tech.htm
 * http://www.robot-electronics.co.uk/files/rpi_md25.c
 *
 */

#pragma once

#include <poll.h>
#include <stdio.h>
#include <uORB/Subscription.hpp>
#include <drivers/device/i2c.h>

#define MCP4725_BASEADDR      0x62
#define MCP4725_CMD_WRITEDAC  0x40
#define MCP4725_CMD_WRITEDACEEPROM 0x60

/**
 * This is a driver for the MD25 motor controller utilizing the I2C interface.
 */
class MCP4725 : public device::I2C
{
public:

	/**
	 * constructor
	 * @param deviceName the name of the device e.g. "/dev/mcp4725"
	 * @param bus the I2C bus
	 * @param address the adddress on the I2C bus
	 * @param speed the speed of the I2C communication
	 */
	MCP4725(const char *deviceName,
	     int bus = 1,
	     int address = MCP4725_BASEADDR,
	     uint32_t speed = 100000);

	/**
	 * deconstructor
	 */
	virtual ~MCP4725() = default;

	/**
	 * @return software version
	 */
	uint8_t getVersion();

	
	int setDeviceAddress(uint8_t address);


	void setVoltage(uint16_t output, bool writeEEPROM);


private:

	uint8_t _version;

};
