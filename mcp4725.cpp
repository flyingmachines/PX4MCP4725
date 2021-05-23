/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file mcp4725.cpp
 *
 *
 */

#include "mcp4725.hpp"
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <systemlib/err.h>
#include <arch/board/board.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>
#include <drivers/drv_hrt.h>


// File descriptors
//static orb_advert_t mavlink_log_pub;

MCP4725::MCP4725(const char *deviceName, int bus,
	   int address, uint32_t speed) :
	I2C("MCP4725", deviceName, bus, address, speed),
	_version(0)
{
	// if initialization fails raise an error, unless
	// probing
	int ret = I2C::init();

	if (ret != OK) {
		printf("I2C::init failed for bus: %d address: %d\n", bus, address);
		//warnc(ret, "I2C::init failed for bus: %d address: %d\n", bus, address);
	}
}

void MCP4725::setVoltage(uint16_t output, bool writeEEPROM)
{
	uint8_t packet[3];

	if (writeEEPROM) {
    	packet[0] = MCP4725_CMD_WRITEDACEEPROM;
  	
  	} else {
    	packet[0] = MCP4725_CMD_WRITEDAC;
  	}
	
	packet[1] = output / 16;        // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
 	packet[2] = (output % 16) << 4;

 	transfer(packet, sizeof(packet), nullptr, 0);

}





