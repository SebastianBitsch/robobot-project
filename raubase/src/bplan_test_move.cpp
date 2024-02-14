#include <string>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"

#include "bplan_test_move.h"


void BPlan20::setup()
{ 

}

BPlan20::~BPlan20()
{
  terminate();
}


void BPlan20::run()
{
	while true {
		mixer.setVelocity(1);
		usleep(2000*1000);
        mixer.setVelocity(0);
		usleep(2000*1000);
	}
}


void BPlan20::terminate()
{ 
}

void BPlan20::toLog(const char* message)
{
  
}

