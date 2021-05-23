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
 * @ file mcp4725_main.cpp
 *
 * Driver for MCP4725
 *
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <parameters/param.h>
#include <arch/board/board.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/adc_report.h>
#include "mcp4725.hpp"

#define PX4_MAX_ADC_CHANNELS 12

static bool thread_should_exit = false;     /**< Deamon exit flag */
static bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
extern "C" __EXPORT int mcp4725_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int mcp4725_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: mcp4725 {start|stop|read|status|search|test|change_address}\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mcp4725_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("mcp4725 already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("mcp4725",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 10,
						 2048,
						 mcp4725_thread_main,
						 (char *const *)argv);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmcp4725 app is running\n");

		} else {
			printf("\tmcp4725 app not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int mcp4725_thread_main(int argc, char *argv[])
{
	printf("[mcp4725] starting\n");

	// if (argc < 5) {
	// 	// extra mcp4725 in arg list since this is a thread
	// 	printf("usage: mcp4725 start bus address\n");
	// 	exit(0);
	// }

	const char *deviceName = "/dev/mcp4725";

	//uint8_t bus = strtoul(argv[3], nullptr, 0);

	//uint8_t address = strtoul(argv[4], nullptr, 0);

	// start
	MCP4725 MCP4725(deviceName);

	thread_running = true;

	int manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int adc_report_sub = orb_subscribe(ORB_ID(adc_report));

	struct manual_control_setpoint_s	manual_control_sp {};
	struct adc_report_s	adcs {};

	uint16_t output = 0;
	float psi = 0.0f;
	float voltagepsi = 0.0f;
	float uctrl = 0.0f;
	int cnt = 0;
	// int cnt1 = 0;

	// loop
	while (!thread_should_exit) {

		bool updated1;


		/* get pilots inputs */
		orb_check(adc_report_sub, &updated1);

		if (updated1) {
		orb_copy(ORB_ID(adc_report), adc_report_sub, &adcs);
		}

		
		for (int j = 0; j < PX4_MAX_ADC_CHANNELS; ++j) {
			if (adcs.channel_id[j] == 15){
					
				if (adcs.channel_value[j] < 0.252f ){
					
					voltagepsi = 0.252f;
				
				}else if(adcs.channel_value[j] > 1.61f){
					
					voltagepsi = 1.61f;
				
				}else{
					voltagepsi = adcs.channel_value[j];
				}

				//Curve Fitting
				psi = 29.46f*voltagepsi - 7.392f;

				//printf("The psi is %0.3f\n", (double)psi);
				//printf("%d %0.3f\n",adcs.channel_id[j],adcs.channel_value[j] );
			
			}
		}

		//--------------------------------------------------------//

		bool updated;

		/* get pilots inputs */
		orb_check(manual_control_sp_sub, &updated);

		if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), manual_control_sp_sub, &manual_control_sp);
		}

		if (manual_control_sp.kill_switch == manual_control_setpoint_s::SWITCH_POS_ON){
			
			float err = 28 - psi;
			uctrl = 4*err;
			float outf = uctrl / 5.0f * 4000.0f;

			if (cnt == 100)
			{
			//printf("the pressure is %0.3f %0.3f\n",(double)psi,(double)outf );
			cnt = 0;
				/* code */
			}

			if (outf > 4000)
			{
				outf = 4095.0f;
			}else if(outf < 0){
				outf = 0.0f;
			}else {}

			output = outf;


			cnt++;
			//output = 4000;

		
		}else{

			//printf("in output 0\n");
			output = 0;
		
		}
		
		bool writeEEPROM = false;
		// if (cnt1 == 100)
		// {
		// 	printf("the output is %0.3u\n",output );
		// 	cnt1 = 0;
		// 		/* code */
		// }
		//cnt1++;

		//output = 2048;
		MCP4725.setVoltage(output, writeEEPROM);

		usleep(10000);
	}

	// exit
	printf("[MD25] exiting.\n");
	thread_running = false;
	return 0;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
