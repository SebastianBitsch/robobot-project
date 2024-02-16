// Bridge rewrite
// Author: Jacob Bechmann Pedersen
// Date: 2022-03-14

#ifndef BRIDGE_H
#define BRIDGE_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <cstring>
#include <mutex>

using namespace std;

class MotorData;
class EventData;

class Bridge{
private:
	static const int MAX_RX_CNT = 500;

public:
	Bridge(string ip, string port);
	~Bridge();
	void sendRaw(const char * msg);
	EventData * event;
	MotorData * motor;

private:
	addrinfo * servinfo;
	int sockfd; // Socket file descriptor
	bool connected;
	mutex sendMtx;
	thread * listener = NULL;

	static void listenLoop(Bridge * bridge); // To spawn the listen loop as a separate thread, it needs to be static
	void decode(char * msg);
};

class MotorData{
public:
	MotorData(Bridge * bridgePtr); // Inherit constructor, just sets bridge
	volatile float velocity[2];
	volatile float current[2];
	void subscribe();

private:
	void decodeVel(char * msg);
  	void decodeCurrent(char * msg);
  	Bridge * bridge;
	friend class Bridge;
};

class EventData{
private:
	static const int MAX_EVENT_FLAGS = 32;

public:
	EventData(Bridge * bridgePtr); // Inherit constructor, just sets bridge
	volatile bool eventFlags[MAX_EVENT_FLAGS]; // 0..31 events are tracked
	void subscribe();

	bool check(int event);
	void awaitAndClear(int event);
	void clear(int event);
	void clearAll();

private:
	void decode(char * msg);
	void set(int event, bool state);
	mutex eventMtx;
  	Bridge * bridge;
	friend class Bridge;

};

class MessageData{
private:
	static const int MAX_EVENT_FLAGS = 32;

public:
	MessageData(Bridge * bridgePtr); // Inherit constructor, just sets bridge
	volatile bool eventFlags[MAX_EVENT_FLAGS]; // 0..31 events are tracked
	void subscribe();

private:
	void decode(char * msg);
	void set(int event, bool state);
	mutex eventMtx;
  	Bridge * bridge;
	friend class Bridge;

};


#endif /*BRIDGE_H*/
