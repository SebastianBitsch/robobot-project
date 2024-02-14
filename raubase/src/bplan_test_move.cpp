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


void BPlanTestMove::setup()
{ 

}

BPlanTestMove::~BPlanTestMove()
{
  terminate();
}


void BPlanTestMove::run()
{
	while true {
		mixer.setVelocity(1);
		usleep(2000*1000);
        mixer.setVelocity(0);
		usleep(2000*1000);
	}
}


void BPlanTestMove::terminate()
{ 
}

void BPlanTestMove::toLog(const char* message)
{
  
}

