#include "mission.h"

MissionHandler::MissionHandler(Bridge * bridgePtr){
	bridge = bridgePtr;
}

void MissionHandler::addLines(const string linesIn){
	stringstream lineStream(linesIn);
	string lineBfr;

	// Go through the lines token by token (\n separates lines):
	while(getline(lineStream, lineBfr, '\n')){
		lines[lineIdx] = lineBfr;
		lineIdx++;
	}
	if (lineIdx > 0)
      sendLines();
}

void MissionHandler::init(){
	bridge->sendRaw("robot stop\n");
	bridge->sendRaw("robot <clear\n");

	// jeg tror vi skal droppe thread-swap
	// dvs stop her

	// Add thread 1 for idle and hardware init:
	bridge->sendRaw("robot <add thread=1\n");

	// Add thread 100 and 101, that switch execution between events being set (30 and 31 respectively). 
	bridge->sendRaw("robot <add thread=100,event=30 : event=31\n");
	// Fill with placeholder mission (finite lines, and can only be modified herafter, not appended to):
	for (int i = 0; i < MAX_MISSION_LINES; i++){
    	bridge->sendRaw("robot <add vel=0 : time=0.1\n");
	}
  
	bridge->sendRaw("robot <add thread=101,event=31 : event=30\n");
	for (int i = 0; i < MAX_MISSION_LINES; i++){
    	bridge->sendRaw("robot <add vel=0 : time=0.1\n");
	}
	usleep(10000); // Allow to commit
}

void MissionHandler::sendLines(){
// skal funktionen selv tilføje
bridge->sendRaw("robot <clear\n");
for (int i = 0; i < lineIdx; i++)
  bridge->sendRaw(lines[i].c_str());  
}
