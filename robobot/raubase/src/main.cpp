
/*
 #***************************************************************************
 #*   Copyright (C) 2023 by DTU
 #*   jcan@dtu.dk
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */

// System libraries
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <cmath>

//
// include local files for data values and functions
#include "uservice.h"
#include "cmixer.h"
#include "sgpiod.h"
#include "bplan20.h"
#include "bplan21.h"
#include "bplan40.h"
#include "bplan100.h"
#include "bplan101.h"
#include "medge.h"
#include "cedge.h"
#include "bplan_test_move.h"
#include "furbs_control.h"

#include "simu.h"
#include "sedge.h"
#include "sdist.h"


#define mes_dist(x) filter_dist = 0; for (int i = 0; i < sampels; i++) { filter_dist += dist.dist[0]/sampels; usleep(filter_dist_wait); printf("filter_dist %i : %f\n", x, filter_dist); }

int main (int argc, char **argv)
{

	// prepare all modules and start data flow
	// but also handle command-line options
	service.setup(argc, argv);
	sedge.setup();
	mixer.setup();

	if (not service.theEnd) { 

		gpio.setPin(16, 1);
		mixer.setVelocity(0.2);
		usleep(1*1000*1000);
		sedge.setSensor(true, true);
		sedge.tick();
		printf("edge raw : %i, %i, %i, %i, %i, %i, %i, %i \n", sedge.edgeRaw[0], sedge.edgeRaw[1], sedge.edgeRaw[2], sedge.edgeRaw[3], sedge.edgeRaw[4], sedge.edgeRaw[5], sedge.edgeRaw[6], sedge.edgeRaw[7]);
		usleep(1*1000*1000);
		printf("edge raw : %i, %i, %i, %i, %i, %i, %i, %i \n", sedge.edgeRaw[0], sedge.edgeRaw[1], sedge.edgeRaw[2], sedge.edgeRaw[3], sedge.edgeRaw[4], sedge.edgeRaw[5], sedge.edgeRaw[6], sedge.edgeRaw[7]);
		usleep(1*1000*1000);
		gpio.setPin(16, 0);
	
	}

	// close all logfiles etc.
	service.terminate();
	return service.theEnd;
}


/*
int main (int argc, char **argv)
{

	// prepare all modules and start data flow
	// but also handle command-line options
	service.setup(argc, argv);
	imu.setup();
	furbs.setup();
	sedge.setup();
	mixer.setup();
	cedge.setup();
	
	int sampels = 20;
	float filter_dist = 0;
	float target_dist = 0.1;
	float float_mes_dist = 0;

	int filter_dist_wait = 21*0000;

	if (not service.theEnd) { 

		gpio.setPin(16, 1);
		{
			auto p = furbs.vel;
			furbs.go_for(3.73, left_line_mode, p);
			furbs.go_for(0.50, no_line_mode, p);
			p.max_vel -= 0.1; //slow down a bit
			furbs.go_for(1.95, left_line_mode, p);
			p.max_vel += 0.1; //regain speed
			
			mes_dist(1);
			
			//Wait till the thing comes by
			while (filter_dist > 0.3) {
				mes_dist(2);
			}

			//Do a distance meassture
			//how far are we away
			mes_dist(3);
			float_mes_dist = filter_dist;
			
			//Go closer
			furbs.go_for(float_mes_dist-target_dist, left_line_mode, p);

			//Now we wait for the thing to go by again
			while (filter_dist > target_dist + 0.1) {
				mes_dist(4);
				float_mes_dist = filter_dist;
			}
			//Once the dist becomes far we go fast
			while (filter_dist < 0.5) {
				mes_dist(5);
			}
			//usleep(1*1000*1000);
			p.max_acc += 0.2;
			p.max_vel += 0.2;
			furbs.go_for(0.5, left_line_mode, p);
			p.max_acc -= 0.2;
			p.max_vel -= 0.2;
			
			furbs.go_for(1.3, left_line_mode, p);

			bool do_box = false;
			if (do_box) {
				mes_dist(6);

				//Once the we find the box
				while (filter_dist > 0.13) {
					furbs.go_for(0.05, left_line_mode, p);
					mes_dist(7);
				}
				
				p.max_vel -= 0.2;
				furbs.go_for(-0.2, no_line_mode, p);
				p.max_vel += 0.2;
				furbs.go_for(0.2, right_line_mode, p);
				furbs.go_for(-1, no_line_mode, p);
				furbs.go_for(0.3, right_line_mode, p);
				furbs.turn(30, p);
			}
			else {
				furbs.go_for(5, right_line_mode, p);
			}
		}
		gpio.setPin(16, 0);
	
	}

	// close all logfiles etc.
	service.terminate();
	return service.theEnd;
}
*/
