
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

enum LineState { on_line, off_line };

/*
void go_to_position (float x, float y) {

	while true {

		pose.x
		pose.y
		pose.heading
		
		mixer.setVelocity();
		mixer.setDesiredHeading();
		usleep(10*1000); //ms before updating velocity and heading'

	}
}
*/

#define mes_dist(x) filter_dist = 0; for (int i = 0; i < sampels; i++) { filter_dist += dist.dist[0]/sampels; usleep(filter_dist_wait); printf("filter_dist %i : %f\n", x, filter_dist); }

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

	float position[3] = {0,0,0};
	float position_target[3] = {0,0,0};

	float direction[3] = {0,0,0};
	float direction_target[3] = {0,0,0};
	bool is_direction_target = false;
	
	LineState line_state = off_line;
	
	int sampels = 20;
	float filter_dist = 0;
	float target_dist = 0.15;
	float float_mes_dist = 0;

	int filter_dist_wait = 21*0000;

	if (not service.theEnd) { 

		gpio.setPin(16, 1);
		{
			auto p = furbs.vel;
			furbs.go_for(3.73, left_line_mode, p);
			furbs.go_for(0.50, no_line_mode, p);
			p.max_vel -= 0.1; //slow down a bit
			furbs.go_for(2, left_line_mode, p);
			p.max_vel += 0.1; //regain speed
			
			mes_dist(1);
			
			//Wait till the thing comes by
			while (filter_dist > 0.3) {
				filter_dist = 0;
				for (int i = 0; i < sampels; i++) {
					filter_dist += dist.dist[0]/sampels;
					usleep(filter_dist_wait); printf("filter_dist2 : %f\n", filter_dist);
				}
			}

			//Do a distance meassture
			//how far are we away
			filter_dist = 0;
			for (int i = 0; i < sampels; i++) {
					filter_dist += dist.dist[0]/sampels;
				usleep(filter_dist_wait); printf("filter_dist3 : %f\n", filter_dist);
			}
			float_mes_dist = filter_dist;
			
			//Go closer
			furbs.go_for(target_dist-float_mes_dist, left_line_mode, p);

			//Now we wait for the thing to go by again
			while (filter_dist > target_dist + 0.1) {
				filter_dist = 0;
				for (int i = 0; i < sampels; i++) {
					filter_dist += dist.dist[0]/sampels;
					usleep(filter_dist_wait); printf("filter_dist4 : %f\n", filter_dist);
				}
				float_mes_dist = filter_dist;
			}
			//Once the dist becomes far we go fast
			while (filter_dist < 0.5) {
				filter_dist = 0;
				for (int i = 0; i < sampels; i++) {
					filter_dist += dist.dist[0]/sampels;
					usleep(filter_dist_wait); printf("filter_dist5 : %f\n", filter_dist);
				}
			}
			//usleep(1*1000*1000);
			p.max_acc += 0.5;
			p.max_vel += 0.4;
			furbs.go_for(1, no_line_mode, p);
			p.max_acc -= 0.5;
			p.max_vel -= 0.4;
			furbs.go_for(1, left_line_mode, p);


		}
		gpio.setPin(16, 0);
		
		//switch(line_state)
		//{
		//	case on_line: 
				//TODO;
		//		break;
		//	case off_line:
				//TODO;
		//		break;
		//}

		//position calculation
			//Get accelerometer (this depends on the direction too)
			//Get line data (if on line)
			//Get wheel data
		
		//Direction calculation
			//Get gyroscope
			//Get line data (if on line)
			//Get wheel data

		//the mission must be none_blocking?
		

		/*
		// all set to go
		// turn on LED on port 16
		// run the planned missions
		plan20.run();
		plan21.run();
		plan40.run();
		plan100.run();
		plan101.run();
		//
		mixer.setVelocity(0.0);
		mixer.setTurnrate(0.0);
		*/

		//sleep(1); // to allow robot to stop		
	}



	//imu.terminate();

	// close all logfiles etc.
	service.terminate();
	return service.theEnd;
}

