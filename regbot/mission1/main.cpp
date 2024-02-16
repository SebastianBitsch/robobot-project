// Mission rewrite
// Author: Jacob Bechmann Pedersen
// Date: 2022-03-14

#include <iostream>
#include <string>
#include <thread>

// Own functions:
#include "bridge.h"
#include "mission.h"

using namespace std;

typedef enum {
	START = 0,
	DRV_FWD = 1,
	END = 0xFF // The final value of the enum
} missionState_t;

void missionFSM(Bridge * bridge, MissionHandler * missionHandler){
	missionState_t missionState = START;
	
	while (missionState != END){
		switch (missionState){
			case START:
				cout << "Now in start state\n";
				missionState = DRV_FWD;
				break;

			case DRV_FWD:
				cout << "Now in drive forward state\n";
				missionHandler->sendLines(
										"vel=0 : ir2 < 0.3\n"
										"vel=0.2,acc=1:dist=0.6\n"
										"vel=0\n"
										": dist=1\n"
										);
				missionHandler->waitToFinish();
				missionState = END;
				break;

			case END:
				cout << "Now in end state\n";
			default:
				break;
		}
	}
}

// The purpose of main() is to create all the modules and run the missions
int main(int argc, char **argv){
	// Setup a bridge connection:
	string bridgeIp = "127.0.0.1";
	string bridgePort = "24001";
	Bridge bridge(bridgeIp, bridgePort); // Spawns a bridge connected to the port and ip
	MissionHandler missionHandler(&bridge); // Creates missionhandler to handle sending missions and checking for events

	// We'll be using these bridge data:
	bridge.motor->subscribe();

	while(1){
		printf("Motor L: %lf - Motor R: %lf\n", bridge.motor->velocity[0], bridge.motor->velocity[1]);
	}

	// Nok en stribe kald til delmissioner

	//missionFSM(&bridge); // Run the mission FSM the rest of the time

	return 0;
}
