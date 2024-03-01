
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
#include "bplan_test_move.h"

#include "simu.h"

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

float max_acc = 1;
float max_vel = 0.7;
float time_interval = 0.1;
float dist_margin = 0.00;
float min_vel = 0.05;

void go_for (float meters) {
	
	float cur_vel = 0;
	float target_vel = max_vel;
	float start[2] = {pose.x, pose.y};
	float dist = 0;

	while (true) {

		dist = sqrt((start[0] - pose.x)*(start[0] - pose.x) + (start[1] - pose.y)*(start[1] - pose.y));

		// Calculate the stopping distance
		float stopping_distance = cur_vel * cur_vel / (2 * max_acc);

		//The distance it will take to reach 0 m/s. A dist_margin is added so it can slow down beforehand.
		if ((meters - dist - dist_margin) <= stopping_distance) {
			target_vel = 0;
		}

		if (cur_vel < target_vel) {
			cur_vel += max_acc * time_interval;
		}
		else if (cur_vel > target_vel) {
			cur_vel -= max_acc * time_interval;
		}

		cur_vel = fmax(min_vel, cur_vel);

		mixer.setVelocity(cur_vel);
		mixer.setDesiredHeading(0);

		float time_interval_usec = time_interval * 1000.0f * 1000.0f;
		usleep((useconds_t)time_interval_usec); //ms before updating velocity and heading
		printf("dist, cur_vel, target_vel,  %f, %f, %f\n", dist, cur_vel, target_vel);
		
		if (dist >= meters) {
			mixer.setVelocity(0);
			break;
		}
	}
}

int main (int argc, char **argv)
{ 
	// prepare all modules and start data flow
	// but also handle command-line options
	service.setup(argc, argv);
	imu.setup();

	float position[3] = {0,0,0};
	float position_target[3] = {0,0,0};

	float direction[3] = {0,0,0};
	float direction_target[3] = {0,0,0};
	bool is_direction_target = false;
	
	LineState line_state = off_line;

	if (not service.theEnd) { 

		gpio.setPin(16, 1);
		go_for(1);
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

