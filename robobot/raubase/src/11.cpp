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
#include "sdist.h"


#include "task11.hpp"
Task11 task11;


void Task11::setup()
{ // ensure there is default values in ini-file
  if (not ini["task11"].has("log"))
  { // no data yet, so generate some default values
    ini["task11"]["log"] = "true";
    ini["task11"]["run"] = "false";
    ini["task11"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["task11"]["print"] == "true";
  //
  if (ini["task11"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_task11.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission task11 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

Task11::~Task11()
{
  terminate();
}

void Task11::run()
{
  if (not setupDone)
    setup();
  if (ini["task11"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 1;
  oldstate = state;
  //
  toLog("Task11 started");
  int count = 0;
  //

while (not finished and not lost and not service.stop)
{
    switch (state)
    {
    case 1:
    if()//detect the crossroads
    pose.resetPose();
    mixer.setVelocity(0);
    mixer.setTurnrate(0.35);
    if(pose.turned==M_PI/4)
    {
    mixer.setTurnrate(0);
    pose.turned=0;
    state=2;
    }
        break;
    
    case 2:
    if (dist.dist[0]<0.1)//detect the gurading robot
        {
        sleep(2);
        mixer.setTurnrate(0.35);//use back wheel go platform
        }
        if(pose.turned==M_PI)
        {
        mixer.setTurnrate(0);
        pose.turned=0;
        mixer.setVelocity(0.3);
        }
        //go up paltform
        if(pose.dist>=0.65)//check whether on platform
        {
        mixer.setVelocity(0);
        pose.dist=0;
        }
    state=3;
        break;
    
    case 3:
    mixer.setTurnrate(0.25);
    if (dist.dist[0]<=0.3&&dist.dist[1]<=0.1)
    {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.05);
    }
    if(dist.dist[0]<=0.1)
    {
        mixer.setVelocity(0);
        pose.dist=0;
        mixer.setTurnrate(0.067);//w=v/r r=45cm
        mixer.setVelocity(0.03);
    }
    if (pose.dist>=2*M_PI*0.45)
    {
        mixer.setTurnrate(0);//w=v/r r=45cm
        mixer.setVelocity(0);
    }

    state=4;
    break;
    

    }
}
}