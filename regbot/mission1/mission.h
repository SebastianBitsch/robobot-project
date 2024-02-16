// Mission line handler
// Author: Jacob Bechmann Pedersen
// Date: 2022-03-14

#ifndef MISSION_H
#define MISSION_H

#include <bits/stdc++.h>
#include "bridge.h"

using namespace std;

class MissionHandler{
private:
	static const int MAX_MISSION_LINES = 20; // Try to trim this, the regbot has limited flash

public:
	MissionHandler(Bridge * bridgePtr);
	void init();
	void addLines(const string linesIn);
	void sendLines();

private:
	string lines[MAX_MISSION_LINES];
	int lineIdx = 0;

	Bridge * bridge;
};


#endif /*MISSION_H*/