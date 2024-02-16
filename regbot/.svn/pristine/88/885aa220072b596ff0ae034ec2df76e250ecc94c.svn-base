// Bridge rewrite
// Author: Jacob Bechmann Pedersen
// Date: 2022-03-14

#include "bridge.h"

// Bridge class:
Bridge::Bridge(string ip, string port){
	// Setup socket to use
	addrinfo hints;
	hints.ai_family = AF_UNSPEC; // don't care IPv4 or IPv6
	hints.ai_socktype = SOCK_STREAM; // TCP stream sockets
	int res;
	if ((res = getaddrinfo(ip.c_str(), port.c_str(), &hints, &servinfo)) != 0){
		fprintf(stderr,"getaddrinfo: %s\n", gai_strerror(res));
	}
	if ((sockfd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol)) == -1){
		fprintf(stderr,"Failed socket connection on: %s:%s\n", ip.c_str(), port.c_str());
	}

	if (connect(sockfd,servinfo->ai_addr, servinfo->ai_addrlen) == -1){
		close(sockfd);
		perror("Bridge connect failed");
		connected = false;
	}
	else{
		connected = true;
	}

	// Initialize the member objects:
	event = new EventData(this);
	motor = new MotorData(this);

	// Start the listener thread:
	listener = new thread(listenLoop, this);
}

Bridge::~Bridge(){
	if (connected){
		connected = false;
		close(sockfd);
	}
	if (servinfo != NULL){
		freeaddrinfo(servinfo);
	}
	printf("Closed socket\n");

	listener->join();
}

void Bridge::sendRaw(const char * msg){
	sendMtx.lock();
	if (connected){
		int len = strlen(msg);
		send(sockfd, msg, len, 0);
	}
	usleep(4000);
	sendMtx.unlock();
}

void Bridge::listenLoop(Bridge * bridge){

	char rxBuf[MAX_RX_CNT];
	memset(rxBuf, '\0', MAX_RX_CNT);
	unsigned int rxCnt = 0;

	while(1){
		char recvChar;
		while (recv(bridge->sockfd, &recvChar, 1, MSG_DONTWAIT) != 1)
		{
			usleep(100);
		}; // Wait until recv gets a character
		
		// Check accepted chars
		if (recvChar >=' ' or recvChar == '\n'){
			rxBuf[rxCnt] = recvChar;

			// If the line ends, terminate string and decode
			if (rxBuf[rxCnt] == '\n'){
				rxBuf[rxCnt] = '\0';
				bridge->decode(rxBuf);
				rxCnt = 0;
				memset(rxBuf, '\0', MAX_RX_CNT);
			}
			else if (rxCnt < MAX_RX_CNT){ // Increment counter
				rxCnt++;
				continue;
			}
			else{ // Buffer overflow
	          printf("Bridge listenLoop overflow\n");
	          rxCnt = 0;
	        }
//	        usleep(1000); // Give buffer some time to fill. 1000 us should be enough for new regbot msg
		}
	}
}

void Bridge::decode(char * msg){
	// Skip leading whitespace
	while (*msg <= ' ' && *msg > '\0'){
		msg++;
	}
	if (*msg != '\0'){

		// if (event->decode(msg)) {}
		// else if (motor->decode(msg)) {}
		// else
		//   printf("#message not understood\n");

		if (strncmp(msg, "event", 5)==0){
			event->decode(msg);
		}
		else if (strncmp(msg, "wve ", 4)==0){ // wheel velocity
			motor->decodeVel(msg);
		}
		else if (strncmp(msg, "mca ", 4)==0){ // motor current
			motor->decodeCurrent(msg);
		}
		else { // not yet supported message - ignore
			printf("Bridge - unhandled message: '%s'\n", msg);
		}
	}
	

}

// MotorData class
MotorData::MotorData(Bridge * bridgePtr){
	bridge = bridgePtr;
}

void MotorData::subscribe(){
	bridge->sendRaw("wve subscribe 2\n"); // velocity
	bridge->sendRaw("mca subscribe 2\n"); // motor current
}

void MotorData::decodeVel(char * msg){
	// dataPtr is set by the end to next char
	// This is a feature of strtof, makes it easy to read the data out
	char * dataPtr = &msg[4];
	velocity[0] = strtof(dataPtr, &dataPtr);
	velocity[1] = strtof(dataPtr, &dataPtr);
}

void MotorData::decodeCurrent(char * msg){
	char * dataPtr = &msg[4];
	current[0] = strtof(dataPtr, &dataPtr);
	current[1] = strtof(dataPtr, &dataPtr);
}

// EventData class
EventData::EventData(Bridge * bridgePtr){
	bridge = bridgePtr;
}

void EventData::subscribe(){
	clearAll();
	bridge->sendRaw("event subscribe 6\n"); // start, stop and mission events
	bridge->sendRaw("event get\n"); // start, stop and mission events
}

bool EventData::check(int event){
	return eventFlags[event];
}

void EventData::awaitAndClear(int event){
	while (eventFlags[event] == false){
		usleep(1000); // Sleep the process 1 ms every poll
	}
	// Once out, clear the flag:
	set(event, false);
}

void EventData::clear(int event){
	set(event, false);
}

void EventData::clearAll(){
	for (int event = 0; event < MAX_EVENT_FLAGS; event++){
		set(event, false);
	}
}

void EventData::decode(char * msg){
	char * dataPtr = &msg[5];
	int event = strtol(dataPtr, &dataPtr, 0);

	// 0 is result of last mission, ignore
	if (event > 0){
		set(event, true);
	}
}

void EventData::set(int event, bool state){
	if (event < MAX_EVENT_FLAGS && event >= 0){
		eventMtx.lock(); // Events must only be accessed by one source at a time (must be locked when set and unset)
		eventFlags[event] = state;
		eventMtx.unlock();
	}
	else{
		fprintf(stderr, "Couldn't set event %d: out of bounds!", event);
	}
}
